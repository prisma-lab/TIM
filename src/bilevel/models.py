import os
import torch
import numpy as np
import lightning.pytorch as pl
import blocks


class EffectPredictor(pl.LightningModule):

    def __init__(self, config):
        super(EffectPredictor, self).__init__()
        self.save_hyperparameters()

        state_dim = config.get("state_dim", 11)
        action_dim = config.get("action_dim", 8)
        effect_dim = config.get("effect_dim", 6)
        hidden_dim = config["hidden_dim"]
        latent_dim = config["latent_dim"]
        n_hidden = config["n_hidden_layers"]
        n_heads = config["n_attention_heads"]
        gumbel_t = config.get("gumbel_t", 1.0)
        gumbel_hard = config.get("gumbel_hard", False)
        batch_norm = config.get("batch_norm", False)
        sigmoid = config.get("sigmoid", False)

        self.lr = config.get("lr", 0.0001)
        self.loss_coeff = config.get("loss_coeff", 1.0)

        enc_layers = [state_dim] + [hidden_dim] * n_hidden + [latent_dim]
        self.encoder = torch.nn.Sequential(
            blocks.MLP(enc_layers, batch_norm=batch_norm, last_layer_norm=True),
            blocks.GumbelSigmoidLayer(hard=gumbel_hard, T=gumbel_t, sigmoid=sigmoid)
        )

        pre_att_layers = [state_dim] + [hidden_dim] * n_hidden
        self.pre_attention = blocks.MLP(pre_att_layers, batch_norm=batch_norm)

        self.attention = blocks.GumbelAttention(
            in_dim=hidden_dim, out_dim=hidden_dim, num_heads=n_heads,
            temperature=gumbel_t, hard=gumbel_hard, sigmoid=sigmoid
        )

        post_enc_layers = [latent_dim + action_dim] + [hidden_dim] * n_hidden
        self.post_encoder = blocks.MLP(post_enc_layers, batch_norm=batch_norm)

        dec_layers = [hidden_dim * n_heads] + [hidden_dim] * n_hidden + [effect_dim]
        self.decoder = blocks.MLP(dec_layers, batch_norm=batch_norm)

    def encode(self, x, eval_mode=False):
        n_sample, n_seg, n_feat = x.shape
        x = x.reshape(-1, n_feat)
        h = self.encoder(x)
        h = h.reshape(n_sample, n_seg, -1)
        if eval_mode:
            h = h.round()
        return h

    def attn_weights(self, x, pad_mask, eval_mode=False):
        n_sample, n_seg, n_feat = x.shape
        x = x.reshape(-1, n_feat)
        x = self.pre_attention(x)
        x = x.reshape(n_sample, n_seg, -1)
        pad_mask = pad_mask.to(x.device)
        attn_weights = self.attention(x, src_key_mask=pad_mask)
        if eval_mode:
            attn_weights = attn_weights.round()
        return attn_weights

    def concat(self, s, a, eval_mode=False):
        h = self.encode(s, eval_mode)
        z = torch.cat([h, a], dim=-1)
        return z

    def aggregate(self, z, attn_weights):
        n_batch, n_seg, n_dim = z.shape
        post_h = self.post_encoder(z.reshape(-1, n_dim)).reshape(n_batch, n_seg, -1).unsqueeze(1)
        att_out = attn_weights @ post_h  # (n_batch, n_head, n_seg, n_dim)
        att_out = att_out.permute(0, 2, 1, 3).reshape(n_batch, n_seg, -1)
        return att_out

    def decode(self, z, pad_mask):
        n_sample, n_seg, z_dim = z.shape
        z = z.reshape(-1, z_dim)
        e = self.decoder(z)
        e = e.reshape(n_sample, n_seg, -1)
        pad_mask = pad_mask.reshape(n_sample, n_seg, 1)
        e_masked = e * pad_mask
        return e_masked

    def forward(self, s, a, pad_mask, eval_mode=False):
        z = self.concat(s, a, eval_mode)
        attn_weights = self.attn_weights(s, pad_mask, eval_mode)
        z_att = self.aggregate(z, attn_weights)
        e = self.decode(z_att, pad_mask)
        return z, attn_weights, e

    def loss(self, e_pred, e, pad_mask):
        loss = torch.nn.functional.mse_loss(e_pred, e, reduction="none")
        loss = (loss * pad_mask.unsqueeze(2)).sum(dim=[1, 2]).mean() * self.loss_coeff
        return loss

    def training_step(self, batch, _):
        s, a, e, pad_mask, _ = batch
        _, _, e_pred = self.forward(s, a, pad_mask)
        loss = self.loss(e_pred, e, pad_mask)
        self.log("train_loss", loss, on_epoch=True, prog_bar=True)
        return loss

    def validation_step(self, batch, _):
        s, a, e, pad_mask, _ = batch
        _, _, e_pred = self.forward(s, a, pad_mask)
        loss = self.loss(e_pred, e, pad_mask)
        self.log("val_loss", loss, on_epoch=True, prog_bar=True)
        return loss

    def test_step(self, batch, _):
        s, a, e, pad_mask, _ = batch
        _, _, e_pred = self.forward(s, a, pad_mask)
        return (e - e_pred).abs()

    def predict_step(self, batch, _):
        s, a, _, pad_mask, sn = batch
        z = self.encode(s, eval_mode=False)
        z = z * pad_mask.unsqueeze(2)
        r = self.attn_weights(s, pad_mask, eval_mode=False)
        zn = self.encode(sn, eval_mode=False)
        zn = zn * pad_mask.unsqueeze(2)
        rn = self.attn_weights(sn, pad_mask, eval_mode=False)
        return {"z": z, "r": r, "a": a, "zn": zn, "rn": rn, "m": pad_mask}

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=self.lr)
        return optimizer

    def on_before_optimizer_step(self, optimizer):
        norms = pl.utilities.grad_norm(self, norm_type=2)
        self.log_dict(norms)


def load_ckpt(name, model_type=EffectPredictor, tag="best"):
    save_dir = os.path.join("logs", name)
    if not os.path.exists(save_dir):
        raise FileNotFoundError(f"Checkpoint directory not found: {save_dir}")

    ckpts = [f for f in os.listdir(save_dir) if f.endswith(".ckpt")]
    if not ckpts:
        raise FileNotFoundError(f"No checkpoint files found in: {save_dir}")

    if tag == "best":
        best_ckpts = [c for c in ckpts if "last" not in c]
        ckpt = best_ckpts[0] if best_ckpts else "last.ckpt"
    else:
        ckpt = "last.ckpt"

    ckpt_path = os.path.join(save_dir, ckpt)
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    model = model_type.load_from_checkpoint(ckpt_path, map_location=device)
    return model, ckpt_path