# Module 0: Linux Basics for Robotics

## Learning Objectives

By the end of this module, you will:
- Navigate the Linux file system using the terminal
- Create, view, edit, and delete files and folders
- Understand file paths, the home directory, and the `~` shortcut
- Use essential keyboard shortcuts that will save you time throughout the workshop
- Connect to the RISA-bot via SSH from your Windows laptop
- Feel confident working in a text-only terminal environment

---

## 1. Why Linux?

The RISA-bot runs **Ubuntu Linux** — the most popular operating system in robotics. Unlike Windows, there is no desktop with icons and folders to click on. Almost everything is done by typing commands into a **terminal** (also called a shell or command line).

Don't worry if this feels unfamiliar! By the end of this module, you will be comfortable enough to navigate, create files, and run programs — everything you need for the rest of the workshop.

---

## 2. Connecting to the Robot via SSH

The robot does not have a monitor, keyboard, or mouse. You control it remotely from your Windows laptop using **SSH** (Secure Shell) — a tool that lets you type commands on your laptop that execute on the robot.

### Step 1 — Open a terminal on your laptop

- Press `Win + R`, type `cmd`, and press Enter. *(Or search for "PowerShell" in the Start menu.)*

### Step 2 — Connect via SSH

```bash
ssh sunrise@192.168.x.x
```
*(Replace `192.168.x.x` with your robot's actual IP address — ask your instructor.)*

- If it asks `Are you sure you want to continue connecting?`, type `yes` and press Enter.
- When prompted for a password, type `risabot` and press Enter.

> [!NOTE]
> **The password is invisible!** When you type the password, nothing appears on screen — no dots, no stars, nothing. This is normal Linux behaviour. Just type it and press Enter.

### Step 3 — You're in!

You should now see something like:

```text
sunrise@risabot:~$
```

This is the robot's **command prompt**. Everything you type here runs on the robot, not on your Windows laptop!

---

## 3. The File System: Where Am I?

Think of the Linux file system as a tree of folders (called **directories**). Here are the three most important commands:

### `pwd` — Print Working Directory (Where am I?)

```bash
pwd
# /home/sunrise
```

This shows your current location. When you first log in, you start in your **home directory** — `/home/sunrise`.

### `ls` — List (What's here?)

```bash
ls
# Desktop  Documents  risabotcar_ws  student_ws
```

This shows all the files and folders in your current directory. Think of it as opening a folder in Windows Explorer.

**Useful variations:**

```bash
ls -l          # Detailed view (size, date, permissions)
ls -la         # Show hidden files too (files starting with .)
ls risabotcar_ws/src/   # Look inside a specific folder without going there
```

### `cd` — Change Directory (Go somewhere)

```bash
cd risabotcar_ws       # Go into the risabotcar_ws folder
cd src                 # Go into src (which is inside risabotcar_ws)
cd ..                  # Go UP one level (back to risabotcar_ws)
cd                     # Go HOME (back to /home/sunrise)
```

**Try it now!** Navigate into the RISA-bot code and back:

```bash
pwd                     # /home/sunrise
cd risabotcar_ws        # Enter the workspace
ls                      # See what's inside
cd src                  # Enter the source folder
ls                      # See the packages
cd                      # Go back home
pwd                     # /home/sunrise — back where you started!
```

---

## 4. The Home Directory and `~`

Your home directory (`/home/sunrise`) is where all your personal files live. Linux provides a shortcut: the **tilde** symbol `~` always means "my home directory."

```bash
cd ~                    # Go home (same as just "cd")
cd ~/risabotcar_ws      # Go to risabotcar_ws from anywhere
ls ~/student_ws/src     # Look inside student_ws/src from anywhere
```

> [!TIP]
> `~` is incredibly useful because it works **from any location**. Instead of typing the full path `/home/sunrise/risabotcar_ws`, you can always type `~/risabotcar_ws`.

---

## 5. Creating and Managing Files

### `mkdir` — Make Directory (Create a folder)

```bash
mkdir my_folder              # Create a folder called my_folder
mkdir -p one/two/three       # Create nested folders in one go
```

The `-p` flag creates parent directories automatically. In Module 1, you will use this to create your workspace:

```bash
mkdir -p ~/student_ws/src    # Creates both student_ws AND src inside it
```

### `touch` — Create an empty file

```bash
touch my_file.py             # Creates an empty Python file
```

### `cat` — View a file's contents

```bash
cat my_file.py               # Print the entire file to the screen
```

Good for small files. For larger files, use:

```bash
less my_file.py              # Scroll through a file (press Q to quit)
```

### `cp` — Copy

```bash
cp file.txt backup.txt              # Copy a file
cp -r my_folder/ my_folder_backup/  # Copy a folder (use -r for directories)
```

### `mv` — Move or Rename

```bash
mv old_name.py new_name.py          # Rename a file
mv file.py ~/student_ws/            # Move file to another folder
```

### `rm` — Remove (Delete)

```bash
rm file.txt                  # Delete a file (no undo! no recycle bin!)
rm -r my_folder/             # Delete a folder and everything inside
```

> [!CAUTION]
> **There is no recycle bin in Linux!** When you `rm` something, it is gone permanently. Always double-check before deleting.

---

## 6. Editing Files with `nano`

Throughout this workshop, you will need to edit Python scripts and configuration files. The easiest terminal text editor is **nano**:

```bash
nano my_file.py
```

This opens the file in a simple editor inside your terminal:

```text
  GNU nano 6.2            my_file.py

#!/usr/bin/env python3
print("Hello, Robot!")




^G Help   ^O Write Out  ^W Where Is   ^K Cut
^X Exit   ^R Read File  ^\ Replace    ^U Paste
```

**Essential nano shortcuts** (the `^` means hold `Ctrl`):

| Shortcut | Action |
|----------|--------|
| `Ctrl + O` | **Save** the file (press Enter to confirm) |
| `Ctrl + X` | **Exit** nano (it will ask to save if you changed anything) |
| `Ctrl + K` | **Cut** the current line |
| `Ctrl + U` | **Paste** the cut line |
| `Ctrl + W` | **Search** for text |
| Arrow keys | Move the cursor around |

**Try it now!** Create and edit a test file:

```bash
nano test.py
```

Type `print("Hello from the robot!")`, then press `Ctrl + O`, `Enter`, `Ctrl + X`. Now run it:

```bash
python3 test.py
# Hello from the robot!
```

---

## 7. Essential Keyboard Shortcuts

These shortcuts will save you enormous amounts of time throughout the workshop:

### In the Terminal

| Shortcut | What It Does |
|----------|-------------|
| `Ctrl + C` | **Stop** a running program (you will use this constantly!) |
| `Ctrl + D` | **Exit** the current terminal/SSH session |
| `Tab` | **Auto-complete** a file or folder name |
| `Tab Tab` | Show all possible completions |
| `↑` / `↓` | Scroll through previous commands (command history) |
| `Ctrl + L` | **Clear** the screen (same as typing `clear`) |
| `Ctrl + R` | **Search** your command history |

### Tab Completion — Your Best Friend

Instead of typing long folder names, type the first few letters and press `Tab`:

```bash
cd risa<Tab>
# Automatically completes to: cd risabotcar_ws/

cd ~/risabotcar_ws/sr<Tab>
# Automatically completes to: cd ~/risabotcar_ws/src/
```

If nothing happens when you press Tab, press `Tab` twice to see all possible matches.

> [!TIP]
> **Use Tab completion constantly!** It prevents typos and saves time. If Tab doesn't complete, it usually means the file or folder doesn't exist — check your spelling.

### Stopping a Running Program

In Modules 2 and 3, you will run programs that keep running until you stop them (like the dashboard or LiDAR driver). To stop them:

- Press **`Ctrl + C`** — this sends a "stop" signal to the program.

This is the single most important shortcut in the workshop!

---

## 8. Running Multiple Terminals

Throughout this workshop, you will often need **two or more terminals** connected to the robot at the same time. For example, in Module 1 you need one terminal running the joystick node and another running your driver.

### Option A: Multiple SSH connections

Open two (or more) separate PowerShell/CMD windows on your laptop, and SSH into the robot in each one:

```text
Window 1: ssh sunrise@192.168.x.x    ← Run the joystick node here
Window 2: ssh sunrise@192.168.x.x    ← Run your driver here
Window 3: ssh sunrise@192.168.x.x    ← Monitor topics here
```

Each window is an independent terminal on the robot.

### Option B: Use `tmux` (advanced)

If your robot has `tmux` installed, you can split one SSH connection into multiple panes:

```bash
tmux                    # Start a new tmux session
Ctrl+B, %               # Split screen vertically
Ctrl+B, ←/→             # Switch between panes
```

For this workshop, Option A (multiple SSH windows) is perfectly fine!

---

## 9. Piping and Filtering

In Module 2, you will encounter commands that use the **pipe** symbol `|`. This sends the output of one command into another:

```bash
ros2 topic list | grep camera
```

This means: "list all ROS topics, then filter to show only lines containing 'camera'."

### `grep` — Search for text

```bash
ros2 topic list | grep camera    # Find topics with "camera" in the name
cat params.yaml | grep speed     # Find lines containing "speed" in a file
```

### `head` — Show only the first few lines

```bash
ros2 topic echo /scan | head -5   # Show only the first 5 lines of output
```

These are optional but very useful for finding information quickly!

---

## 10. Understanding File Paths

Throughout the workshop, you will see two types of file paths:

### Absolute paths (start from the root `/`)

```text
/home/sunrise/risabotcar_ws/src/risabot_automode/config/params.yaml
```

This is the **full address** — it works from anywhere.

### Relative paths (start from where you are now)

```text
src/risabot_automode/config/params.yaml
```

This only works if you are currently inside `/home/sunrise/risabotcar_ws/`.

### Key path symbols

| Symbol | Meaning | Example |
|--------|---------|---------|
| `~` | Home directory (`/home/sunrise`) | `cd ~/student_ws` |
| `.` | Current directory | `ls .` (same as `ls`) |
| `..` | Parent directory (one level up) | `cd ..` |
| `/` | Root of the entire file system | `ls /` |

---

## 11. Quick Reference Card

Keep this handy during the workshop!

### Navigation
```bash
pwd                      # Where am I?
ls                       # What's here?
cd folder_name           # Go into a folder
cd ..                    # Go up one level
cd ~                     # Go home
```

### Files and Folders
```bash
mkdir -p folder/sub      # Create folders
nano file.py             # Edit a file
cat file.py              # View a file
cp file.py copy.py       # Copy
mv old.py new.py         # Rename/move
rm file.py               # Delete (careful!)
```

### Running Programs
```bash
python3 script.py        # Run a Python script
Ctrl + C                 # Stop a running program
```

### Workshop-Specific
```bash
ssh sunrise@192.168.x.x                   # Connect to robot
source /opt/ros/humble/setup.bash          # Load ROS 2
source ~/risabotcar_ws/install/setup.bash  # Load RISA-bot packages
colcon build                               # Build your code
ros2 run package_name node_name            # Run a ROS 2 node
ros2 topic list                            # See all active topics
ros2 topic echo /topic_name                # Listen to a topic
```

---

## 12. Hands-On: Explore the Robot

Now let's practice! Connect to the robot via SSH and try these exercises:

### Exercise 1: Navigate the File System

```bash
# 1. Check where you are
pwd

# 2. List everything in your home directory
ls ~

# 3. Go into the RISA-bot workspace
cd ~/risabotcar_ws

# 4. List the source packages
ls src/

# 5. Find the params.yaml file
ls src/risabot_automode/config/

# 6. View the first 20 lines of the config file
head -20 src/risabot_automode/config/params.yaml

# 7. Go back home
cd ~
```

### Exercise 2: Create Your Own File

```bash
# 1. Create a test folder in your home directory
mkdir ~/test_folder

# 2. Go into it
cd ~/test_folder

# 3. Create a Python script
nano hello.py
```

Type this in nano:
```python
print("Hello from RISA-bot!")
print("I am learning Linux!")
```

Save (`Ctrl + O`, `Enter`) and exit (`Ctrl + X`). Now run it:

```bash
python3 hello.py
```

### Exercise 3: Practice Keyboard Shortcuts

1. Press `↑` to recall your last command without retyping it.
2. Type `cd ~/risa` and press `Tab` — it should auto-complete to `cd ~/risabotcar_ws/`.
3. Run `python3 hello.py` again, then press `Ctrl + C` while it's running (it will finish instantly since it's a short script — but now you know the shortcut!).
4. Type `ls /home/sunrise/risabotcar_ws/src/risabot_automode/` — now try doing the same thing with Tab completion. Much faster!

### Exercise 4: Clean Up

```bash
# Delete the test folder (practice rm -r)
rm -r ~/test_folder

# Verify it's gone
ls ~
```

---

## 13. What You've Learned

You now know the essential Linux skills needed for the rest of the workshop:

| Skill | Commands |
|-------|----------|
| **Navigate** | `pwd`, `ls`, `cd`, `cd ..`, `cd ~` |
| **Create** | `mkdir`, `touch`, `nano` |
| **View** | `cat`, `less`, `head` |
| **Manage** | `cp`, `mv`, `rm` |
| **Control** | `Ctrl + C` (stop), `Tab` (complete), `↑` (history) |
| **Connect** | `ssh sunrise@IP` |

You are ready to start working with ROS 2!

---

**Next:** [Module 1 — Introduction to ROS 2](01-introduction-to-ros.md)
