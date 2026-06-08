# Bilevel Planning with Learned Symbolic Abstractions

This repository contains the code for the paper *"Bilevel Planning with Learned Symbolic Abstractions from Interaction Data"*.

## Installation

### Setup

```bash
# Clone the repository
git clone https://github.com/fatihdogangun/bilevel-planning.git
cd bilevel-planning

# Create conda environment
conda create -n bilevel python=3.10 -y
conda activate bilevel

# Install packages
pip install -e .

# Install Fast Downward
git clone https://github.com/aibasel/downward.git
cd downward && ./build.py
cd ..
```

## Usage

### 1. Data Collection

Collect interaction data through random exploration:

```bash
python collect_mp.py -s explore.py -d data/<dataset_name> -N 25000 -T 10 -p 8 -n_min 2 -n_max 4
```

### 2. Training

Train the model:

```bash
python train.py -c configs/train.yaml
```

### 3. Rule Learning

Learn symbolic operators and generate PDDL and PPDDL domains:

```bash
python learn_rules.py -n <model_name>
```

### 4. Evaluation

```bash
python evaluate.py -c configs/eval.yaml
```


## Pre-trained Resources

- **Interaction data** — `data/`
- **Model checkpoints** — `logs/`
- **Generated domains** — `save/`
