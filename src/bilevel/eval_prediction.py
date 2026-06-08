import argparse
import torch
from models import EffectPredictor, load_ckpt
from dataset import StateActionEffectDataset

parser = argparse.ArgumentParser("Evaluate effect prediction error.")
parser.add_argument("models", nargs="+")
parser.add_argument("--dataset", default="blocks")
args = parser.parse_args()

device = "cpu"

avg_errors = []
for name in args.models:
    model, ckpt_path = load_ckpt(name, model_type=EffectPredictor, tag="best")
    model.freeze()
    model.to(device)

    dataset = StateActionEffectDataset(args.dataset, split="test")
    loader = torch.utils.data.DataLoader(dataset, batch_size=128)
    errors = []
    for sample in loader:
        error = model.test_step(sample, None)
        errors.append(error)

    errors = torch.cat(errors, dim=0)
    avg_error = errors.mean(dim=[0, 1]).sum()
    avg_errors.append(avg_error)

    print(f"{name}: {avg_error * 100:.3f} +- {errors.std(dim=[0, 1]).sum() * 100:.3f} cm")

if len(avg_errors) > 1:
    avg_errors = torch.stack(avg_errors, dim=0)
    print(f"\nOverall: {avg_errors.mean(dim=0) * 100:.3f} +- {avg_errors.std(dim=0) * 100:.3f} cm")