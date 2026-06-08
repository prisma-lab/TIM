import os

import torch
import lightning.pytorch as pl


class StateActionEffectDM(pl.LightningDataModule):
    def __init__(self, name, batch_size=128, num_workers=0):
        super().__init__()
        self.name = name
        self.batch_size = batch_size
        self.num_workers = num_workers

    def prepare_data(self):
        self.data_path = os.path.join("data", self.name)
        self.train_set = StateActionEffectDataset(self.name, split="train")
        self.val_set = StateActionEffectDataset(self.name, split="val")

    def train_dataloader(self):
        return torch.utils.data.DataLoader(self.train_set, batch_size=self.batch_size,
                                           num_workers=self.num_workers, shuffle=True)

    def val_dataloader(self):
        return torch.utils.data.DataLoader(self.val_set, batch_size=self.batch_size,
                                           num_workers=self.num_workers)


class StateActionEffectDataset(torch.utils.data.Dataset):
    def __init__(self, name, split="train"):
        path = os.path.join("data", name)
        self.state = torch.load(os.path.join(path, "state.pt"))
        self.action = torch.load(os.path.join(path, "action.pt"))
        self.effect = torch.load(os.path.join(path, "effect.pt"))
        self.mask = torch.load(os.path.join(path, "mask.pt"))
        self.post_state = torch.load(os.path.join(path, "post_state.pt"))

        n_train = int(len(self.state) * 0.8)
        n_val = int(len(self.state) * 0.1)
        if split == "train":
            self.state = self.state[:n_train]
            self.action = self.action[:n_train]
            self.effect = self.effect[:n_train]
            self.mask = self.mask[:n_train]
            self.post_state = self.post_state[:n_train]
        elif split == "val":
            self.state = self.state[n_train:n_train + n_val]
            self.action = self.action[n_train:n_train + n_val]
            self.effect = self.effect[n_train:n_train + n_val]
            self.mask = self.mask[n_train:n_train + n_val]
            self.post_state = self.post_state[n_train:n_train + n_val]
        elif split == "test":
            self.state = self.state[n_train + n_val:]
            self.action = self.action[n_train + n_val:]
            self.effect = self.effect[n_train + n_val:]
            self.mask = self.mask[n_train + n_val:]
            self.post_state = self.post_state[n_train + n_val:]

    def __len__(self):
        return len(self.state)

    def __getitem__(self, idx):
        state = self.state[idx]
        a = self.action[idx]
        post_state = self.post_state[idx]
        n_objects, _ = state.shape
        action = torch.zeros(n_objects, 8, dtype=torch.float)
        action[a[0], :4] = torch.tensor([1, a[2], a[3], a[6]], dtype=torch.float)
        action[a[1], 4:] = torch.tensor([1, a[4], a[5], a[7]], dtype=torch.float)
        effect = torch.cat([self.effect[idx][:, :3], self.effect[idx][:, 7:10]], dim=-1)
        mask = torch.zeros(n_objects, dtype=torch.float)
        mask[:self.mask[idx]] = 1.0

        return state, action, effect, mask, post_state