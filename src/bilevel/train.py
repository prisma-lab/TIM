
import argparse
import os
import yaml
import lightning.pytorch as pl
from lightning.pytorch.loggers import TensorBoardLogger
from models import EffectPredictor, load_ckpt
from dataset import StateActionEffectDM

parser = argparse.ArgumentParser("Train EffectPredictor.")
parser.add_argument("-c", "--config", help="config file", type=str, required=True)
parser.add_argument("--resume", action="store_true", help="resume from latest checkpoint")
args = parser.parse_args()

with open(args.config, "r") as f:
    config = yaml.safe_load(f)

ckpt_callback = pl.callbacks.ModelCheckpoint(dirpath=os.path.join("logs", config["name"]),
                                             save_last=True, save_top_k=1, monitor="val_loss",
                                             mode="min")

#logger = TensorBoardLogger("logs", name=config["name"])

trainer = pl.Trainer(max_epochs=config["epoch"], gradient_clip_val=10.0,
                      devices=config["devices"], callbacks=[ckpt_callback])

ckpt_path = None
if args.resume:
    model, ckpt_path = load_ckpt(config["name"], tag="latest")
else:
    model = EffectPredictor(config)

dm = StateActionEffectDM(config["dataset_name"], batch_size=config["batch_size"])
trainer.fit(model, datamodule=dm, ckpt_path=ckpt_path)