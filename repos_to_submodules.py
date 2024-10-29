import glob
import subprocess
import yaml
from pathlib import Path
from typing import Dict
import logging
import argparse

logger = logging.getLogger("repos_to_submodules")
logging.basicConfig(level=logging.INFO)


def get_submodules(path: Path = Path("src")) -> Dict[str, Dict]:
    all_submodules = {}
    # Get all submodules within the directory (all .repo files)
    for repo_file in glob.glob(f"{path}/**/*.repos", recursive=True):
        with open(repo_file, "r") as f:
            submodules = yaml.safe_load(f)["repositories"]
            # Add the path to the submodule
            for submodule in submodules.values():
                submodule["path"] = Path(repo_file).parent
            all_submodules.update(submodules)
            del submodules
    return all_submodules


def is_submodule(repo_name: str) -> bool:
    try:
        subprocess.check_output(
            ["git", "submodule", "status", repo_name], stderr=subprocess.DEVNULL
        )
        return True
    except subprocess.CalledProcessError as e:
        return False


def add_submodule(branch: str, url: str, path: str, dry_run: bool):
    if is_submodule(path):
        logger.warning(f"Submodule {path} already exists, skipping")
    else:
        if dry_run:
            logger.info(f"git submodule add -b {branch} {url} {path}")
        else:
            logger.info(f"Cloning {path} from {url} using branch {branch}")
            subprocess.call(["git", "submodule", "add", "-b", branch, url, path])


def repos_to_submodules(submodules: Dict[str, Dict], dry_run: bool):
    for repo_name, repo_info in submodules.items():
        branch = repo_info["version"]
        url = repo_info["url"]
        path = repo_info["path"] / repo_name
        add_submodule(branch, url, path.as_posix(), dry_run)


def main(dry_run: bool = False):
    logger.info("Starting to clone submodules")
    # Get all submodules from src and third_party
    submodules = get_submodules(Path("src"))
    submodules.update(get_submodules(Path("third_party")))
    # Clone all submodules
    if submodules:
        repos_to_submodules(submodules, dry_run)


if __name__ == "__main__":
    args = argparse.ArgumentParser(description="Repos to submodules")
    args.add_argument("--dry-run", action="store_true", help="Dry run")
    args = args.parse_args()
    if args.dry_run:
        logger.info("Running in dry-run mode")
    main(args.dry_run)
