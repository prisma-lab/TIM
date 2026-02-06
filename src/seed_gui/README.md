# SEED GUI

A PyQt5-based graphical interface for visualizing the **SEED (Social Emotional Entity Descriptor)** Working Memory.  
This tool allows users to monitor belief hierarchies, emphasis levels, and emotional states in real time through an interactive tree and graph representation.

---

## 📂 Project Structure

To comply with ROS 2 standards for a Python package named `seed_gui`, organize your workspace as follows:

```text
seed_gui/
├── package.xml          # Package dependencies and metadata
├── setup.py             # ROS 2 entry point configuration
├── setup.cfg            # Script installation paths
├── resource/
│   └── seed_gui         # Marker file for the index
└── seed_gui/
    ├── __init__.py      # Python package marker
    └── seed_gui.py      # Main application logic
```

---

## 🛠 Installation & Build Procedure

Navigate to your ROS 2 workspace:

```bash
cd ~/dev_ws/src
```

Create the folder structure:

```bash
mkdir -p seed_gui/seed_gui seed_gui/resource
touch seed_gui/seed_gui/__init__.py
touch seed_gui/resource/seed_gui
```

Copy the code:

Save your Python source code as:

```text
seed_gui/seed_gui/seed_gui.py
```

Build the package:

```bash
cd ~/dev_ws
colcon build --packages-select seed_gui
```

Source the workspace:

```bash
source install/setup.bash
```

---

## 🚀 Usage

This node requires a **mandatory command-line argument** representing the name of the SEED instance.  
This argument is stored as a global variable to identify the agent being monitored.

### Command Syntax

```bash
ros2 run seed_gui seed_gui <SEED_INSTANCE_NAME>
```

### Example

To monitor a SEED instance named `agent_01`:

```bash
ros2 run seed_gui seed_gui agent_01
```

---

## ✨ Features

### 🗂 Hierarchical Tree View
A structured list showing:
- Belief names  
- Emphasis levels  
- Releaser status  
- Goal status  
- Truth values  

### 🧠 Interactive Graph View
A dynamic topological map of the agent’s memory.

**Node Types**
- 🔵 Blue — Goals  
- 🟢 Green — Active Releasers  
- ⚪ Gray / 🔴 Red — Standard / Inactive nodes  

**Layout**
- Automatic horizontal and vertical spacing  
- Prevents node overlap in complex hierarchies  

### 🔍 Detailed Inspection
Selecting a node in either the tree or graph opens a side panel displaying the full JSON data for that belief.

### 🖥 Command Console
An integrated input bar for sending raw text commands directly to the SEED agent.

### 🧭 Navigation
- Mouse-based panning  
- `Ctrl + Scroll` zooming within the graph view  

---

## 📡 ROS 2 Interface

### Subscribed Topics

- `/seed_<SEED_INSTANCE_NAME>/wm` (`std_msgs/String`)  
  Receives a JSON-encoded string containing the current belief map.

### Published Topics

- `/seed_<SEED_INSTANCE_NAME>/stream` (`std_msgs/String`)  
  Publishes user-entered commands from the GUI to the SEED agent.

### Parameters / Arguments

- **SEED Instance Name** (positional argument)  
  Used to contextually identify the agent within the UI.

## Acknowledgments

This project was originally developed by **Andrea Pinto** as part of his Bachelor's thesis.  
His initial design and implementation laid the foundation for the SEED GUI, enabling the visualization and interaction with SEED Working Memory structures.  
This work builds upon and extends his original contribution.