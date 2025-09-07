#!/usr/bin/env python3

import os
import sys
import subprocess
import time
import shutil
from pathlib import Path

try:
    import inquirer
except ImportError:
    print("❌ Error: 'inquirer' module not found!")
    print("Please install it using:")
    print("  source venv/bin/activate")
    print("  pip install inquirer")
    print("\nOr refer to docs/013_virtual_environment_setup.md")
    sys.exit(1)

class RobotisWorkspaceCLI:
    def __init__(self):
        self.workspace_root = Path(__file__).parent.absolute()
        self.build_dir = self.workspace_root / "build"
        self.install_dir = self.workspace_root / "install"
        self.log_dir = self.workspace_root / "log"
        
    def clear_screen(self):
        os.system('clear' if os.name == 'posix' else 'cls')
        
    def print_header(self):
        print("🤖 " + "="*60)
        print("   ROBOTIS OP3 Workspace CLI Tool")
        print("   Workspace:", str(self.workspace_root))
        print("="*62)
        print()
        
    def check_workspace(self):
        src_dir = self.workspace_root / "src"
        if not src_dir.exists():
            print("❌ Error: This doesn't appear to be a ROS 2 workspace!")
            print(f"Missing 'src' directory in {self.workspace_root}")
            return False
        return True
        
    def run_command(self, command, description):
        print(f"🔨 {description}")
        print(f"📝 Running: {' '.join(command)}")
        print("-" * 50)
        
        start_time = time.time()
        
        try:
            # Change to workspace directory
            os.chdir(self.workspace_root)
            
            # Run command with real-time output
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            # Print output in real-time
            while True:
                output = process.stdout.readline()
                if output == '' and process.poll() is not None:
                    break
                if output:
                    print(output.strip())
                    
            return_code = process.poll()
            elapsed_time = time.time() - start_time
            
            print("-" * 50)
            if return_code == 0:
                print(f"✅ {description} completed successfully!")
                print(f"⏱️  Time elapsed: {elapsed_time:.2f} seconds")
            else:
                print(f"❌ {description} failed with return code: {return_code}")
                print(f"⏱️  Time elapsed: {elapsed_time:.2f} seconds")
                
            return return_code == 0
            
        except KeyboardInterrupt:
            print("\n🛑 Build interrupted by user")
            return False
        except Exception as e:
            print(f"❌ Error running command: {e}")
            return False
    
    def clean_build_dirs(self):
        dirs_to_clean = [self.build_dir, self.install_dir, self.log_dir]
        cleaned = []
        
        for dir_path in dirs_to_clean:
            if dir_path.exists():
                try:
                    shutil.rmtree(dir_path)
                    cleaned.append(dir_path.name)
                except Exception as e:
                    print(f"⚠️  Warning: Could not remove {dir_path}: {e}")
                    
        if cleaned:
            print(f"🧹 Cleaned directories: {', '.join(cleaned)}")
        else:
            print("✨ Workspace already clean (no build artifacts found)")
            
    def find_and_remove_nested_git(self, root_dir):
        removed_dirs = []
        
        print(f"🔍 Scanning for nested .git directories in: {root_dir}")
        
        for root, dirs, files in os.walk(root_dir):
            if '.git' in dirs:
                git_path = os.path.join(root, '.git')
                try:
                    shutil.rmtree(git_path)
                    removed_dirs.append(git_path)
                    print(f"   ✅ Removed: {git_path}")
                except Exception as e:
                    print(f"   ❌ Error removing {git_path}: {e}")
        
        return removed_dirs
        
    def git_cleanup_menu(self):
        print("🗂️ Git Repository Cleanup")
        print()
        
        src_dir = self.workspace_root / "src"
        if not src_dir.exists():
            print("❌ No 'src' directory found in workspace")
            input("Press Enter to continue...")
            return
            
        # Scan for nested git repositories
        nested_git_dirs = []
        print("🔍 Scanning for nested git repositories...")
        
        for root, dirs, files in os.walk(src_dir):
            if '.git' in dirs:
                git_path = os.path.join(root, '.git')
                nested_git_dirs.append(git_path)
                
        if not nested_git_dirs:
            print("✨ No nested .git directories found")
            input("Press Enter to continue...")
            return
            
        print(f"📂 Found {len(nested_git_dirs)} nested .git directories:")
        for git_dir in nested_git_dirs:
            rel_path = os.path.relpath(git_dir, self.workspace_root)
            print(f"   📁 {rel_path}")
        print()
        
        git_options = [
            "🗑️ Remove all nested .git directories",
            "📋 Show detailed information", 
            "🔙 Back to main menu"
        ]
        
        questions = [
            inquirer.List('git_option',
                         message='Select git cleanup operation',
                         choices=git_options)
        ]
        
        try:
            answers = inquirer.prompt(questions)
            if not answers:
                return
                
            selected = answers['git_option']
            print()
            
            if selected.startswith("🗑️ Remove all"):
                # Warning and confirmation
                print("⚠️  WARNING: This will permanently delete all nested .git directories!")
                print("💡 This converts nested repositories into regular directories")
                print("💡 You will lose git history and connection to upstream repos")
                print()
                
                confirm_questions = [
                    inquirer.Confirm('confirm_remove',
                                   message='Are you sure you want to remove all nested .git directories?',
                                   default=False)
                ]
                
                confirm_answers = inquirer.prompt(confirm_questions)
                if confirm_answers and confirm_answers['confirm_remove']:
                    removed = self.find_and_remove_nested_git(src_dir)
                    
                    print()
                    print("🎯 Operation completed!")
                    print(f"📊 Total .git directories removed: {len(removed)}")
                    
                    if removed:
                        print()
                        print("📋 Removed directories:")
                        for git_dir in removed:
                            rel_path = os.path.relpath(git_dir, self.workspace_root)
                            print(f"   ✅ {rel_path}")
                            
                    print()
                    print("💡 Your workspace is now a single monorepo!")
                    
                else:
                    print("🛑 Operation cancelled")
                    
            elif selected.startswith("📋 Show detailed"):
                print("📊 Detailed git repository information:")
                print()
                
                for git_dir in nested_git_dirs:
                    parent_dir = os.path.dirname(git_dir)
                    rel_parent = os.path.relpath(parent_dir, self.workspace_root)
                    
                    print(f"📁 {rel_parent}/")
                    print(f"   🗂️ Git directory: {os.path.relpath(git_dir, self.workspace_root)}")
                    
                    try:
                        # Try to get git info if git command is available
                        original_cwd = os.getcwd()
                        os.chdir(parent_dir)
                        
                        try:
                            # Get current branch
                            result = subprocess.run(['git', 'branch', '--show-current'], 
                                                  capture_output=True, text=True, timeout=5)
                            if result.returncode == 0 and result.stdout.strip():
                                print(f"   🌿 Current branch: {result.stdout.strip()}")
                                
                            # Get remote info
                            result = subprocess.run(['git', 'remote', '-v'], 
                                                  capture_output=True, text=True, timeout=5)
                            if result.returncode == 0 and result.stdout.strip():
                                remotes = [line.split()[:2] for line in result.stdout.strip().split('\n') if line.strip()]
                                if remotes:
                                    print(f"   🌐 Remote: {remotes[0][1]}")
                                    
                        except (subprocess.TimeoutExpired, FileNotFoundError):
                            print(f"   ℹ️  Git info unavailable")
                            
                        os.chdir(original_cwd)
                        
                    except Exception as e:
                        print(f"   ❌ Error reading git info: {e}")
                        
                    # Calculate directory size
                    try:
                        total_size = sum(f.stat().st_size for f in Path(parent_dir).rglob('*') if f.is_file())
                        size_mb = total_size / (1024 * 1024)
                        print(f"   💾 Size: {size_mb:.1f} MB")
                    except:
                        print(f"   💾 Size: Unable to calculate")
                        
                    print()
                    
            elif selected.startswith("🔙 Back"):
                return
                
            if not selected.startswith("🔙 Back"):
                print()
                input("Press Enter to continue...")
            
        except KeyboardInterrupt:
            print("\n🛑 Operation cancelled")
            return
            
    def clean_workspace_menu(self):
        print("🧹 Clean Workspace Options")
        print()
        
        # Show current status
        status_info = []
        total_size = 0
        
        for dir_path in [self.build_dir, self.install_dir, self.log_dir]:
            if dir_path.exists():
                try:
                    size = sum(f.stat().st_size for f in dir_path.rglob('*') if f.is_file())
                    size_mb = size / (1024 * 1024)
                    status_info.append(f"{dir_path.name}: {size_mb:.1f} MB")
                    total_size += size_mb
                except:
                    status_info.append(f"{dir_path.name}: exists")
            else:
                status_info.append(f"{dir_path.name}: not found")
        
        if status_info:
            print("📊 Current workspace status:")
            for info in status_info:
                print(f"   {info}")
            if total_size > 0:
                print(f"   Total size: {total_size:.1f} MB")
            print()
        
        clean_options = [
            "🧹 Clean all (build + install + log)",
            "🔨 Clean build directory only", 
            "📦 Clean install directory only",
            "📝 Clean log directory only",
            "🔍 Show detailed directory info",
            "🔙 Back to main menu"
        ]
        
        questions = [
            inquirer.List('clean_option',
                         message='Select clean operation',
                         choices=clean_options)
        ]
        
        try:
            answers = inquirer.prompt(questions)
            if not answers:
                return
                
            selected = answers['clean_option']
            print()
            
            if selected.startswith("🧹 Clean all"):
                # Confirmation for full clean
                confirm_questions = [
                    inquirer.Confirm('confirm_clean',
                                   message='Are you sure you want to clean all build artifacts?',
                                   default=False)
                ]
                
                confirm_answers = inquirer.prompt(confirm_questions)
                if confirm_answers and confirm_answers['confirm_clean']:
                    self.clean_build_dirs()
                else:
                    print("🛑 Clean operation cancelled")
                    
            elif selected.startswith("🔨 Clean build"):
                if self.build_dir.exists():
                    try:
                        shutil.rmtree(self.build_dir)
                        print(f"🧹 Cleaned: {self.build_dir.name}")
                    except Exception as e:
                        print(f"❌ Error removing {self.build_dir}: {e}")
                else:
                    print(f"✨ {self.build_dir.name} directory not found")
                    
            elif selected.startswith("📦 Clean install"):
                if self.install_dir.exists():
                    try:
                        shutil.rmtree(self.install_dir)
                        print(f"🧹 Cleaned: {self.install_dir.name}")
                    except Exception as e:
                        print(f"❌ Error removing {self.install_dir}: {e}")
                else:
                    print(f"✨ {self.install_dir.name} directory not found")
                    
            elif selected.startswith("📝 Clean log"):
                if self.log_dir.exists():
                    try:
                        shutil.rmtree(self.log_dir)
                        print(f"🧹 Cleaned: {self.log_dir.name}")
                    except Exception as e:
                        print(f"❌ Error removing {self.log_dir}: {e}")
                else:
                    print(f"✨ {self.log_dir.name} directory not found")
                    
            elif selected.startswith("🔍 Show detailed"):
                print("📁 Detailed directory information:")
                print()
                
                for dir_path in [self.build_dir, self.install_dir, self.log_dir]:
                    print(f"📂 {dir_path.name}/")
                    if dir_path.exists():
                        try:
                            # Count files and subdirectories
                            files = list(dir_path.rglob('*'))
                            file_count = len([f for f in files if f.is_file()])
                            dir_count = len([f for f in files if f.is_dir()])
                            
                            # Calculate total size
                            total_size = sum(f.stat().st_size for f in files if f.is_file())
                            size_mb = total_size / (1024 * 1024)
                            
                            print(f"   📊 Files: {file_count}, Directories: {dir_count}")
                            print(f"   💾 Size: {size_mb:.2f} MB ({total_size:,} bytes)")
                            print(f"   📍 Path: {dir_path}")
                            
                            # Show largest files
                            file_sizes = [(f, f.stat().st_size) for f in files if f.is_file()]
                            if file_sizes:
                                largest_files = sorted(file_sizes, key=lambda x: x[1], reverse=True)[:3]
                                print(f"   🔍 Largest files:")
                                for file_path, size in largest_files:
                                    size_mb = size / (1024 * 1024)
                                    rel_path = file_path.relative_to(dir_path)
                                    print(f"      {size_mb:.1f} MB - {rel_path}")
                                    
                        except Exception as e:
                            print(f"   ❌ Error reading directory: {e}")
                    else:
                        print(f"   ✨ Directory does not exist")
                    print()
                    
            elif selected.startswith("🔙 Back"):
                return
                
            if not selected.startswith("🔙 Back") and not selected.startswith("🔍 Show"):
                print()
                input("Press Enter to continue...")
            
        except KeyboardInterrupt:
            print("\n🛑 Operation cancelled")
            return
            
    def get_custom_worker_count(self):
        while True:
            try:
                questions = [
                    inquirer.Text('workers', message='Enter number of parallel workers (1-16)')
                ]
                answers = inquirer.prompt(questions)
                worker_count = int(answers['workers'])
                
                if 1 <= worker_count <= 16:
                    return worker_count
                else:
                    print("❌ Please enter a number between 1 and 16")
            except (ValueError, TypeError):
                print("❌ Please enter a valid number")
            except KeyboardInterrupt:
                return None
                
    def colcon_build_menu(self):
        print("🔨 Colcon Build Options")
        print()
        
        build_options = [
            "Basic build (colcon build)",
            "Symlink install (colcon build --symlink-install)",
            "Single worker + symlink (--parallel-workers 1)",
            "Dual worker + symlink (--parallel-workers 2)", 
            "Custom worker count + symlink",
            "Clean build (remove build/install/log first)",
            "Back to main menu"
        ]
        
        questions = [
            inquirer.List('build_option',
                         message='Select build option',
                         choices=build_options)
        ]
        
        try:
            answers = inquirer.prompt(questions)
            if not answers:
                return
                
            selected = answers['build_option']
            print()
            
            # Clean build option
            if selected.startswith("Clean build"):
                self.clean_build_dirs()
                print()
                
                # Ask for build type after cleaning
                clean_build_options = [
                    "Basic build",
                    "Symlink install", 
                    "Single worker + symlink",
                    "Dual worker + symlink",
                    "Custom worker count + symlink"
                ]
                
                questions = [
                    inquirer.List('clean_build_option',
                                 message='Select build option after cleaning',
                                 choices=clean_build_options)
                ]
                
                clean_answers = inquirer.prompt(questions)
                if not clean_answers:
                    return
                selected = clean_answers['clean_build_option']
            
            # Build command construction
            if selected.startswith("Basic build"):
                command = ["colcon", "build"]
                description = "Basic colcon build"
                
            elif selected.startswith("Symlink install"):
                command = ["colcon", "build", "--symlink-install"]
                description = "Colcon build with symlink install"
                
            elif selected.startswith("Single worker"):
                command = ["colcon", "build", "--symlink-install", "--parallel-workers", "1"]
                description = "Colcon build with 1 worker"
                
            elif selected.startswith("Dual worker"):
                command = ["colcon", "build", "--symlink-install", "--parallel-workers", "2"] 
                description = "Colcon build with 2 workers"
                
            elif selected.startswith("Custom worker"):
                worker_count = self.get_custom_worker_count()
                if worker_count is None:
                    return
                command = ["colcon", "build", "--symlink-install", "--parallel-workers", str(worker_count)]
                description = f"Colcon build with {worker_count} workers"
                
            elif selected.startswith("Back"):
                return
            else:
                return
                
            # Execute build command
            success = self.run_command(command, description)
            
            print()
            input("Press Enter to continue...")
            
        except KeyboardInterrupt:
            print("\n🛑 Operation cancelled")
            return
            
    def main_menu(self):
        while True:
            self.clear_screen()
            self.print_header()
            
            if not self.check_workspace():
                input("Press Enter to exit...")
                break
                
            main_options = [
                "🔨 Colcon Build Options",
                "🧹 Clean Workspace", 
                "🗂️ Git Repository Cleanup",
                "🚀 Launch Robot (Coming Soon)",
                "📋 Package Management (Coming Soon)", 
                "🔧 Workspace Tools (Coming Soon)",
                "❌ Exit"
            ]
            
            questions = [
                inquirer.List('main_option',
                             message='Select operation',
                             choices=main_options)
            ]
            
            try:
                answers = inquirer.prompt(questions)
                if not answers:
                    break
                    
                selected = answers['main_option']
                
                if selected.startswith("🔨"):
                    self.colcon_build_menu()
                elif selected.startswith("🧹"):
                    self.clean_workspace_menu()
                elif selected.startswith("🗂️"):
                    self.git_cleanup_menu()
                elif selected.startswith("❌"):
                    print("👋 Goodbye!")
                    break
                else:
                    print("🚧 Feature coming soon!")
                    input("Press Enter to continue...")
                    
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                break

def main():
    cli = RobotisWorkspaceCLI()
    cli.main_menu()

if __name__ == "__main__":
    main()