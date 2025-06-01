# ✨ This guide explains how to run the Webots simulation and Python controller on your own machine

--------------------

## ⚙️ System Requirements

* Windows 10 or higher
* Python 3.8+
* Webots (tested with version R2025a)
* Internet access (for initial Webots asset loading)

--------------------

## 🧰 1. Install Webots

Download and install Webots from the official website:

\*\*→ \*\*[**https://cyberbotics.com/#download**](https://cyberbotics.com/#download)

Install it in the default location.

--------------------

## 🐍 2. Set Up the Python Environment

In your project root (`robot-ai-simulation/`), create and activate a virtual environment:
```bash
python -m venv .venv
.venv\Scripts\activate
```

Create a file at:
```
.venv/Lib/site-packages/webots_controller.pth
```

And add the following line (if `Webots` is located in your `Users` folder, otherwise adapt the path):
```
C:\Users\<your-username>\AppData\Local\Programs\Webots\lib\controller\python
```

Set the following environment variable **permanently**:
* **Variable name:** `WEBOTS_HOME`
* **Value:** `C:\Users\<your-username>\AppData\Local\Programs\Webots` (if `Webots` is located in your `Users` folder, otherwise adapt the path)

--------------------

## 🧪 3. Download the needed libraries

You will need to install python libraries in your project environment:
```
.\venv\Scripts\activate
pip install -r requirements.txt
```

--------------------

## 🌍 4. Run the Simulation in Webots

1. Open the Webots application
2. Go to: `File → Open World`
3. Select: ```robots/e-puck/worlds/my_world.wbt```
4. Press the **Play** button (▶️)
The Robot should be executing the program located in `robots/e-puck/controllers/basic/basic.py` and you should see this at the beginning of the console:
```
WARNING: E-puck "e-puck": The 'e-puck' robot window library has not been found.
WARNING: E-puck "e-puck": The remote control library has not been found.
INFO: basic: Starting controller: python.exe -u basic.py
```

--------------------

## ✅ 5. Success !

You now have:

* A local `E-puck` robot simulation using your own PROTO and assets
* All the Python libraries needed
* A clean, extensible project architecture

You're ready to experiment !