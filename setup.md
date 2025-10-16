# ✨ This guide explains how to run the Webots simulation and Python controller on your own machine

--------------------

## ⚙️ System Requirements

* Windows 10 or higher
* Python 3.8+
* Webots (tested with version R2025a)
* Internet access (for initial Webots asset loading)

--------------------

## 🧰 1. Install Webots

Download and install Webots from the [**official website**](https://cyberbotics.com/#download)

Install it in the default location.

--------------------

## 🐍 2. Set Up the Python Environment

In your project root (`robot-ai-simulation/`), create and activate a virtual environment:
```bash
Set-ExecutionPolicy RemoteSigned -Scope CurrentUser
python -m venv .venv
.venv\Scripts\activate
```

And you will also need to add your `Webots Path` to your `PYTHONPATH` system variable. To do so, just edit your 'system environment variables', in the System Variables:
- if `PYTHONPATH` already exist add `;Webots Path` at the end
- if not, create a new variable namde `PYTHONPATH` and put the value `Webots Path` (default value `C:\Users\<YOUR_NAME>\AppData\Local\Programs\Webots\lib\controller\python`)

--------------------

## 🧪 3. Set Up the used modules/libraries

You will need to install python libraries in your project environment:
```
.\venv\Scripts\activate
pip install -r requirements.txt
```

--------------------

## 🌍 4. Run the Simulation in Webots

Type in the base project console ```.\launch.bat``` (you can use a world name to modify the world used).  
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