import math
import torch

class Linear(torch.nn.Module):
    def __init__(self, in_features, out_features, std=None, batch_norm=False, gain=None):
        super(Linear, self).__init__()
        self.in_features = in_features
        self.out_features = out_features
        self.weight = torch.nn.Parameter(torch.Tensor(out_features, in_features))
        self.bias = torch.nn.Parameter(torch.Tensor(out_features))
        if batch_norm:
            self.batch_norm = torch.nn.BatchNorm1d(num_features=self.out_features)

        if std is not None:
            self.weight.data.normal_(0., std)
            self.bias.data.normal_(0., std)
        else:
            if gain is None:
                gain = 1
            stdv = math.sqrt(gain / self.weight.size(1))
            self.weight.data.normal_(0., stdv)
            self.bias.data.zero_()

    def forward(self, x):
        x = torch.nn.functional.linear(x, self.weight, self.bias)
        if hasattr(self, "batch_norm"):
            x = self.batch_norm(x)
        return x

    def extra_repr(self):
        return "in_features={}, out_features={}".format(self.in_features, self.out_features)


class NormedLinear(torch.nn.Module):
    def __init__(self, in_features, out_features):
        super(NormedLinear, self).__init__()
        self.in_features = in_features
        self.out_features = out_features
        self.weight = torch.nn.Parameter(torch.Tensor(out_features, in_features))
        stdv = math.sqrt(1 / self.weight.size(1))
        self.weight.data.normal_(0., stdv)

    def forward(self, x):
        x = 3 * torch.nn.functional.normalize(x, dim=-1)
        wn = 3 * torch.nn.functional.normalize(self.weight, dim=-1)
        x = torch.nn.functional.linear(x, wn)
        return x

    def extra_repr(self):
        return "in_features={}, out_features={}".format(self.in_features, self.out_features)


class MLP(torch.nn.Module):
    def __init__(self, layer_info, activation=torch.nn.ReLU(), std=None, batch_norm=False,
                 indrop=None, hiddrop=None, last_layer_norm=False):
        super(MLP, self).__init__()
        layers = []
        in_dim = layer_info[0]
        for i, unit in enumerate(layer_info[1:-1]):
            if i == 0 and indrop:
                layers.append(torch.nn.Dropout(indrop))
            elif i > 0 and hiddrop:
                layers.append(torch.nn.Dropout(hiddrop))
            layers.append(Linear(in_features=in_dim, out_features=unit, std=std,
                                 batch_norm=batch_norm, gain=2))
            layers.append(activation)
            in_dim = unit
        if last_layer_norm:
            layers.append(NormedLinear(in_features=in_dim, out_features=layer_info[-1]))
        else:
            layers.append(Linear(in_features=in_dim, out_features=layer_info[-1]))
        self.layers = torch.nn.Sequential(*layers)

    def forward(self, x):
        return self.layers(x)


def sample_gumbel_diff(*shape):
    eps = 1e-20
    u1 = torch.rand(shape)
    u2 = torch.rand(shape)
    diff = torch.log(torch.log(u2 + eps) / torch.log(u1 + eps) + eps)
    return diff


def gumbel_sigmoid(logits, T=1.0, hard=False, sigmoid=False):
    g = sample_gumbel_diff(*logits.shape)
    g = g.to(logits.device)
    if sigmoid:
        y = logits / T
    else:
        y = (g + logits) / T
    s = torch.sigmoid(y)
    if hard:
        s_hard = s.round()
        s = (s_hard - s).detach() + s
    return s


class GumbelSigmoidLayer(torch.nn.Module):
    def __init__(self, hard=False, T=1.0, sigmoid=False):
        super(GumbelSigmoidLayer, self).__init__()
        self.hard = hard
        self.T = T
        self.sigmoid = sigmoid

    def forward(self, x):
        return gumbel_sigmoid(x, self.T, self.hard, self.sigmoid)


class GumbelAttention(torch.nn.Module):
    def __init__(self, in_dim, out_dim, num_heads, temperature=1.0, hard=False, sigmoid=False):
        super(GumbelAttention, self).__init__()
        self.num_heads = num_heads
        self.in_dim = in_dim
        self.out_dim = out_dim
        self._denom = math.sqrt(out_dim)
        self.wq = torch.nn.Parameter(torch.nn.init.xavier_normal_(
            torch.Tensor(num_heads, out_dim, in_dim)
        ))
        self.wk = torch.nn.Parameter(torch.nn.init.xavier_normal_(
            torch.Tensor(num_heads, out_dim, in_dim)
        ))
        self.hard = hard
        self.t = temperature
        self.sigmoid = sigmoid

    def forward(self, x, src_key_mask=None):
        if src_key_mask is None:
            src_key_mask = torch.ones(x.shape[0], x.shape[1], dtype=torch.float, device=x.device)
        batch, token, dim = x.shape
        x = x.reshape(batch * token, 1, dim, 1)
        wq = self.wq.unsqueeze(0)
        wk = self.wk.unsqueeze(0)
        pad_mask = src_key_mask.reshape(batch, token, 1, 1)
        q = (wq @ x).reshape(batch, token, self.num_heads, -1) * pad_mask
        q = q.permute(0, 2, 1, 3)
        q = 3 * torch.nn.functional.normalize(q, dim=-1)
        k = (wk @ x).reshape(batch, token, self.num_heads, -1) * pad_mask
        k = k.permute(0, 2, 1, 3)
        k = 3 * torch.nn.functional.normalize(k, dim=-1)
        attn = (q @ k.permute(0, 1, 3, 2))
        binarized_attn = gumbel_sigmoid(attn, self.t, self.hard, self.sigmoid)
        pad_mask = src_key_mask.reshape(batch, token, 1) @ src_key_mask.reshape(batch, 1, token)
        binarized_attn = binarized_attn * pad_mask.unsqueeze(1)
        return binarized_attn