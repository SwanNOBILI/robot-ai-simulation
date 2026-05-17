import argparse
import subprocess
import sys
import os



# Webots executable path
WEBOTS_EXE = r"C:\Users\nobil\AppData\Local\Programs\Webots\msys64\mingw64\bin\webots.exe"
# World files
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
WORLDS_DIR  = os.path.join(SCRIPT_DIR, "robots", "e_puck", "worlds")
WORLD_TRAIN = os.path.join(WORLDS_DIR, "my_world.wbt")
WORLD_EVAL  = os.path.join(WORLDS_DIR, "my_world.wbt")  # will become comparison_world.wbt



def launch_webots(world_path: str, headless: bool = False) -> int:
    # Check if Webots is found
    if not os.path.isfile(WEBOTS_EXE):
        print(f"ERROR: Webots executable not found at:\n  {WEBOTS_EXE}")
        sys.exit(1)
    # Check if World file is found
    if not os.path.isfile(world_path):
        print(f"ERROR: World file not found at:\n  {world_path}")
        sys.exit(1)
    cmd = [WEBOTS_EXE]
    # Default configuration
    if headless:
        cmd += ["--batch", "--no-rendering", "--mode=fast"]
    cmd.append(world_path)
    # Give PYTHONPATH to Webots
    env = os.environ.copy()
    env["PYTHONPATH"] = SCRIPT_DIR + os.pathsep + env.get("PYTHONPATH", "")
    # Launchign Webots
    print(f"Launching Webots {'(headless)' if headless else '(GUI)'}...")
    result = subprocess.run(cmd, env=env)
    return result.returncode



def run_train(episodes: int) -> None:
    # Run `episodes` training episodes back-to-back in headless mode.
    # Each episode is a full Webots simulation (Webots quits at the end via supervisor.simulationQuit()).
    print(f"---------- TRAINING MODE ({episodes} episode(s)) ----------\n")
    for episode in range(1, episodes + 1):
        print(f"--- Episode {episode}/{episodes} ---")
        code = launch_webots(WORLD_TRAIN, headless=True)
        if code != 0:
            print(f"WARNING: Webots exited with code {code} on episode {episode}.")
    print("\nTraining complete.")

def run_eval(basic: bool, sac: bool) -> None:
    # Run an evaluation episode in GUI mode.
    # - basic only  : uses my_world.wbt with the basic controller
    # - basic + sac : will use comparison_world.wbt (not yet implemented)
    print("---------- EVALUATION MODE ----------\n")
    if basic and not sac:
        print("Evaluating: basic controller")
        launch_webots(WORLD_EVAL, headless=False)
    elif basic and sac:
        # TODO: switch to comparison_world.wbt once it is ready
        print("Evaluating: basic + SAC controllers")
        print("NOTE: comparison_world.wbt is not yet implemented.")
        print("      Running basic controller only for now.")
        launch_webots(WORLD_EVAL, headless=False)
    else:
        print("ERROR: --eval requires at least --basic.")
        print("  Usage examples:")
        print("    python main.py --eval --basic")
        print("    python main.py --eval --basic --sac")
        sys.exit(1)
    print("\nEvaluation complete.")



# Parser
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Orchestrator for the e-puck Webots simulation.",
                                     formatter_class=argparse.RawTextHelpFormatter,
                                     epilog=(
                                            "Examples:\n"
                                            "  python main.py --train --episodes 100\n"
                                            "  python main.py --eval --basic\n"
                                            "  python main.py --eval --basic --sac\n"
                                     )
    )

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--train", action="store_true", help="Run N training episodes in headless mode (fast, no GUI).")
    mode.add_argument("--eval",  action="store_true", help="Run one evaluation episode in GUI mode.")
    parser.add_argument("--episodes", type=int, default=1, metavar="N", help="Number of training episodes (only with --train). Default: 1.")
    parser.add_argument("--basic", action="store_true", help="Include the basic controller in evaluation.")
    parser.add_argument("--sac",   action="store_true", help="Include the SAC controller in evaluation.")
    return parser.parse_args()



# Main loop
if __name__ == "__main__":
    args = parse_args()
    if args.train:
        if args.episodes < 1:
            print("ERROR: --episodes must be >= 1.")
            sys.exit(1)
        run_train(args.episodes)
    elif args.eval:
        run_eval(basic=args.basic, sac=args.sac)