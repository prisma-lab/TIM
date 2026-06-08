import time
import os
import argparse
import subprocess
import multiprocessing as mp
import torch


def collect(script, num, t, folder, idx, n_min=None, n_max=None):
    cmd = ["python", script, "-N", num, "-T", t, "-o", folder, "-i", idx]
    if n_min is not None:
        cmd += ["-n_min", str(n_min)]
    if n_max is not None:
        cmd += ["-n_max", str(n_max)]
    subprocess.run(cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Collect interaction data in parallel.")
    parser.add_argument("-s", help="script", type=str, required=True)
    parser.add_argument("-d", help="data folder", type=str, required=True)
    parser.add_argument("-N", help="number of data per proc", type=int, required=True)
    parser.add_argument("-T", help="interaction per episode", type=int, required=True)
    parser.add_argument("-p", help="number of procs", type=int, required=True)
    parser.add_argument("-n_min", help="minimum number of objects", type=int, default=2)
    parser.add_argument("-n_max", help="maximum number of objects", type=int, default=4)
    args = parser.parse_args()

    if not os.path.exists(args.d):
        os.makedirs(args.d)

    procs = []
    start = time.time()
    for i in range(args.p):
        p = mp.get_context("spawn").Process(target=collect, args=[args.s, str(args.N), str(args.T), args.d, str(i)])
        p.start()
        procs.append(p)

    for i in range(args.p):
        procs[i].join()
    end = time.time()
    elapsed = end - start
    print(f"Collected {args.p*args.N} samples in {elapsed:.2f} seconds. {args.p*args.N/elapsed}")
    print("Merging rolls...")
    keys = ["action", "effect", "mask", "state", "post_state"]
    for key in keys:
        field = []
        for i in range(args.p):
            field.append(torch.load(os.path.join(args.d, f"{key}_{i}.pt")))
        field = torch.cat(field, dim=0)
        torch.save(field, os.path.join(args.d, f"{key}.pt"))
        for i in range(args.p):
            os.remove(os.path.join(args.d, f"{key}_{i}.pt"))
    print("Done.")

