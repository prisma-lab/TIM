# ln-to-crf-ros2-interface

## Overview

The **ln-to-crf-ros2-interface** package provides a bridge between the ROS 2 ecosystem and the LN communication layer used by the Human Factory Interface (HFI).

It acts as a middleware translation layer that exposes ROS 2 topics and services through LN, enabling real-time visualization and control of robotic systems from a web-based GUI.

This package is fully responsible for both ROS 2 ↔ LN topic forwarding and ROS 2 ↔ LN service bridging.

---

## Main Responsibilities

* Translate **ROS 2 Topics → LN Topics**
* Translate **ROS 2 Services → LN Services**
* Stream robot state and planner data to the HFI
* Enable remote execution of robot actions from the HFI
* Provide a unified communication layer between ROS 2 and external interfaces

---

## Architecture

The entire **ln-to-crf-ros2-interface** package contains both ROS 2 integration logic and the LN communication layer. The Human Factory Interface (HFI) is an external system that consumes LN topics and services.

```text id="x8kq3p"
┌────────────────────────────────────────────────────────────────────┐
│               ln-to-crf-ros2-interface (ROS 2 Package)           │
│                                                                    │
│  ┌──────────────────────┐        ┌─────────────────────────────┐  │
│  │     ROS 2 Nodes      │        │      ROS 2 Services         │  │
│  │                      │        │                             │  │
│  │ • Joint States       │        │ • Learn DMP Skill           │  │
│  │ • Sequence Planner   │        │ • Execute Skill             │  │
│  │ • Robot Status       │        │ • Robot Actions             │  │
│  └──────────┬───────────┘        └──────────┬──────────────────┘  │
│             │                                │                     │
│             ▼                                ▼                     │
│      ┌────────────────┐            ┌─────────────────┐            │
│      │ Topic Manager  │            │ Service Manager │            │
│      └───────┬────────┘            └───────┬─────────┘            │
│              │                             │                      │
│              └──────────────┬──────────────┘                      │
│                             ▼                                     │
│                    ┌──────────────────┐                            │
│                    │     LN Stack     │                            │
│                    │                  │                            │
│                    │  LN Topics       │                            │
│                    │  LN Services     │                            │
│                    └────────┬─────────┘                            │
└──────────────────────────────┼─────────────────────────────────────┘
                               │
                               │ Network Communication
                               ▼
                ┌──────────────────────────────────┐
                │     Human Factory Interface      │
                │            (Web GUI)             │
                └──────────────────────────────────┘
                               │
                               │
               ┌───────────────┴────────────────┐
               │                                │
               ▼                                ▼
        Visualization                    User Actions
     • Joint States                    • Learn Skill
     • Planner Results                 • Execute Skill
     • Robot Status                    • Robot Commands
```

---

## Components

### Topic Manager

The **Topic Manager** handles translation of ROS 2 topics into LN topics.

It enables external systems (such as the HFI) to subscribe to and visualize robot and application data in real time.

Typical forwarded topics include:

* Robot joint states
* Robot status information
* Sequence planner outputs
* Task execution feedback
* Any configured ROS 2 topic streams

The Topic Manager subscribes to ROS 2 topics and republishes them through LN.

---

### Service Manager

The **Service Manager** exposes ROS 2 services through the LN layer.

It allows external interfaces to trigger robot behavior and higher-level functionality.

Typical services include:

* Learning skills using Dynamic Movement Primitives (DMP)
* Executing learned skills
* Starting robot actions
* Stopping robot actions
* Executing task-specific workflows

Incoming LN service requests are translated into ROS 2 service calls, and responses are forwarded back through LN.

---

## Data Flow

### Topic Flow

```text id="zq9m4t"
ROS 2 Publisher
      │
      ▼
ROS 2 Topic
      │
      ▼
Topic Manager
      │
      ▼
LN Topic
      │
      ▼
HFI Visualization
```

---

### Service Flow

```text id="p3n8qd"
HFI User Action
      │
      ▼
LN Service Request
      │
      ▼
Service Manager
      │
      ▼
ROS 2 Service
      │
      ▼
Robot Execution
```

---

## Human Factory Interface (HFI)

The **Human Factory Interface (HFI)** is a web-based GUI that consumes LN topics and services exposed by this package.

### Visualization Capabilities

* Robot joint states
* Robot status
* Sequence planner results
* Execution feedback
* Real-time system monitoring

### Control Capabilities

* Trigger skill learning (DMP)
* Execute learned skills
* Start/stop robot behaviors
* Trigger ROS 2 services through LN

---

## Package Structure

```text id="q9w1ld"
ln-to-crf-ros2-interface/
├── topic_manager/
│   └── ROS 2 → LN topic translation layer
│
├── service_manager/
│   └── ROS 2 ↔ LN service translation layer
│
├── launch/
│   └── System launch files
│
└── config/
    └── Topic/service mapping configuration
```

---

## How to Run

To start the **ln-to-crf-ros2-interface**, use the provided ROS 2 launch system.

All launch files are located in the `launch/` directory.

### Launch the full interface

```bash
ros2 launch ln-to-crf-ros2-interface ln_to_crf_ros2_interface_bringup.launch.py
```

This will start:

* Topic Manager node
* Service Manager node
* LN communication layer
* All configured topic and service bridges

Make sure your ROS 2 environment is sourced before running:

```bash
source install/setup.bash
```

---

## Summary

The **ln-to-crf-ros2-interface** package is a middleware bridge between ROS 2 and the Human Factory Interface (HFI).

It encapsulates both topic and service translation logic, exposing:

* ROS 2 topics as LN topics via the **Topic Manager**
* ROS 2 services as LN services via the **Service Manager**

This enables:

* Real-time robot monitoring in a web interface
* Remote execution of robot functionalities
* A clean separation between ROS 2 systems and external applications
