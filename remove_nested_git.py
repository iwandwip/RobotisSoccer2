#!/usr/bin/env python3
import os
import shutil
import sys

def find_and_remove_nested_git(root_dir):
    """Find and remove all nested .git directories"""
    removed_dirs = []
    
    for root, dirs, files in os.walk(root_dir):
        if '.git' in dirs:
            git_path = os.path.join(root, '.git')
            try:
                shutil.rmtree(git_path)
                removed_dirs.append(git_path)
                print(f"Removed: {git_path}")
            except Exception as e:
                print(f"Error removing {git_path}: {e}")
    
    return removed_dirs

if __name__ == "__main__":
    # Set the directory to search (current directory by default)
    search_dir = "src" if os.path.exists("src") else "."
    
    print(f"Searching for nested .git directories in: {search_dir}")
    print("This will permanently delete all nested .git directories!")
    
    # Ask for confirmation
    confirm = input("Are you sure you want to proceed? (y/N): ").strip().lower()
    if confirm != 'y':
        print("Operation cancelled.")
        sys.exit(0)
    
    # Find and remove nested .git directories
    removed = find_and_remove_nested_git(search_dir)
    
    print(f"\nOperation completed!")
    print(f"Total .git directories removed: {len(removed)}")
    
    if removed:
        print("\nRemoved directories:")
        for dir_path in removed:
            print(f"  - {dir_path}")