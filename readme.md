# 🧠 Project: Simulated Autonomous Mobile Robot with AI (Webots + PyTorch)

--------------------------------------------------------------------------------


## 🎯 Main Objective

Develop a **simulated autonomous robot** in **Webots**, controlled by an **AI model (MLP, LSTM, or GRU)** trained using **simulated sensor data**. The entire project is software-based and runs on a **low-end computer without a GPU**.

--------------------

## 🧩 Project Steps (big picture)

1. **Install and get started with Webots**
   - Free 3D robotics simulator
   - Configure for lightweight use (no GPU required)
   - Control an existing robot (e.g., e-puck, TurtleBot) with Python

2. **Basic robot control**
   - Move robot using speed and rotation commands
   - Read from simulated sensors: distance, collision, position

3. **Data collection**
   - Log sensor inputs + actions taken
   - Use sequential format: `.csv` or structure suitable for RNNs

4. **AI training (PyTorch on CPU)**
   - Train a model:
     - **MLP**: input → action
     - **LSTM/GRU**: sensor sequences → predicted action
   - Validate on a test set

5. **Deploy AI in Webots**
   - Load the trained model
   - Use it in real-time to control the robot inside the simulation

6. **Evaluation & Comparison**
   - Metrics: distance traveled, collisions, goal completion
   - Compare:
     - Basic (rule-based) control
     - Random behavior
     - Learned AI behavior

--------------------

## 🧠 Technologies Used

| Tool / Language     | Purpose                                       |
|---------------------|-----------------------------------------------|
| Webots              | Robotics simulation                           |
| Python              | Main programming language (rapid prototyping) |
| PyTorch (CPU only)  | Train neural networks (MLP, LSTM, GRU)        |
| NumPy / Matplotlib  | Data processing and visualization             |

--------------------

## ⚠️ Constraints

- **No GPU** → must run on **CPU only**
- **No physical hardware**: simulation only
- Must use **free and open-source tools**
- Long-term goal: get closer to **embedded system constraints** (real-time, low memory)

--------------------

## 🔄 Language Strategy: Python ➜ C++ Transition (optional)

> To better reflect embedded systems practices, I will consider a **progressive transition to C++**:

- Start with **Python** for easier development, testing, and AI training
- Structure code to allow rewriting performance-critical parts in **C++**
- Reuse Webots but switch to a **C++ controller** instead of Python
- Export the trained model (e.g., to ONNX) and use a **C++ runtime** (like ONNX Runtime or a lightweight embedded inference library)

--------------------

## 🏣 Folders Architecture

```
robot-ai-simulation/
│
├── ai-scripts/             'AI-related scripts'
│   ├── model.py                'MLP, LSTM, GRU, ... architectures'
│   ├── parser.py               'Use input parameters when running the script'
│   ├── test.py                 'Testing logic'
│   ├── train.py                'Training logic'
│   └── utils.py                'Other miscellanous functions'
│
├── data/                   'Sensor data and logs from simulations'
│   ├── raw/                    'Raw collected data'
│   └── processed/              'Cleaned, normalized, ready-for-training data'
│
├── docs/                   'Documentation, architecture diagrams, ...'
│
├── logs/                   'Notes, experiment logs, debug info'
│   └── log_n.md              'Log number "n" on the work done'
│
├── models/                 'Saved PyTorch models (".pt" or ".onnx" files)'
│   └── readme.md               'Notes on each model and training context'
│
├── robots/e-puck           'All (e-puck) Webot-related files'
│   ├── controllers/            'C/C++/Python scripts that make the Robot work'
│   ├── proto/                  'Robot configuration file + some other useful folders'
│   └── worlds/                 'The Worlds (simulation) configuration (".wbt" files)'
│
├── .gitignore              'Files ignored by Git'
│
├── main.py                 'Entry-point or test runner'
│
├── readme.md               'Project Description'
│
└── requirements.txt        'Python dependencies'
```
--------------------

## 💾 Version Control: Using GitHub Effectively

- 🔄 **Commit regularly** after each meaningful step:
  - “Set up Webots environment”
  - “First working robot motion”
  - “Logged sensor data successfully”
  - “Trained MLP model”
- 📅 This gives:
  - Clear project history
  - Safe backup in case of system failure
  - Professional appearance for your GitHub profile
  - Easier debugging and rollbacks
- 🗂️ I will write a small dev log (`log.md`) and updating the `README.md` progressively

**Branching strategy (optional):**
- `main`: always stable
- `dev-*` branches for experimentation


--------------------------------------------------------------------------------

# ✅ Learning Goals

- Master **robotic simulation**
- Apply **sequence-based AI** to robotics
- Use **AI training frameworks** like PyTorch on CPU
- Use **GitHub efficiently** with meaningful commits
- Transition progressively from **Python to C++** (optional)

--------------------------------------------------------------------------------