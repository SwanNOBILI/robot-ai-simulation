import argparse
import subprocess
import sys
import os



# Webots executable path
WEBOTS_EXE = r"C:\Users\nobil\AppData\Local\Programs\Webots\msys64\mingw64\bin\webots.exe"
# World files
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORLDS_DIR = os.path.join(SCRIPT_DIR, "robots", "e_puck", "worlds")
BASIC_WORLD = os.path.join(WORLDS_DIR, "world_basic.wbt")
SAC_WORLD = os.path.join(WORLDS_DIR, "world_sac.wbt")
COMPARISON_WORLD = os.path.join(WORLDS_DIR, "world_basic_sac.wbt")    # TODO



def launch_webots(world_path: str, headless: bool = False, mode = "train", episodes = None) -> int:
    # Check if Webots is found
    if not os.path.isfile(WEBOTS_EXE):
        print(f"ERROR: Webots executable not found at:\n  {WEBOTS_EXE}")
        sys.exit(1)
    # Check if World file is found
    if not os.path.isfile(world_path):
        print(f"ERROR: World file not found at:\n  {world_path}")
        sys.exit(1)
    # Manage Environment Variable
    env = os.environ.copy()
    env["PYTHONPATH"] = SCRIPT_DIR + os.pathsep + env.get("PYTHONPATH", "")
    env["MODE"] = mode
    if episodes is not None: env["TOTAL_EPISODES"] = str(episodes)
    # Manage Configuration
    cmd = [WEBOTS_EXE]
    if headless: cmd += ["--batch", "--no-rendering", "--mode=fast"]
    cmd.append(world_path)
    # Launching Webots
    print(f"Launching Webots {'HEADLESS' if headless else 'GUI'}")
    result = subprocess.run(cmd, env=env)
    return result.returncode



def run_train(episodes: int) -> None:
    code = launch_webots(SAC_WORLD, headless=True, mode="train", episodes=str(episodes))
    if code != 0: print(f"WARNING: Webots exited with code {code}")
    print("Training complete !")

def run_eval(basic: bool, sac: bool) -> None:
    # Run an evaluation episode in GUI mode.
    # - basic only  : uses world_basic.wbt with the basic controller
    # - sac only  : uses world_sac.wbt with the basic controller
    # - basic + sac : will use world_basic_sac.wbt
    print("---------- EVALUATION MODE ----------\n")
    if basic and not sac:
        print("Evaluating: \"basic\" controller")
        launch_webots(BASIC_WORLD, headless=False, mode="eval")
    elif sac and not basic:
        print("Evaluating: \"sac\" controller")
        launch_webots(SAC_WORLD, headless=False, mode="eval")
    elif basic and sac:
        print("Evaluating: basic + SAC controllers")
        print("NOTE: comparison_world.wbt is not yet implemented.")
        print("      Running basic controller only for now.")
        launch_webots(COMPARISON_WORLD, headless=False, mode="eval")
    else:
        print("ERROR: --eval requires a controller")
        print("  Usage examples:")
        print("    python main.py --eval --basic")
        print("    python main.py --eval --sac")
        print("    python main.py --eval --basic --sac")
        sys.exit(1)
    print("Evaluation complete")



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