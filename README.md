# Matlab Git Integration Guide
# beginning: download the git on https://git-scm.com/
- `!git --version` % confirm that you successfully installed git on the computer
- % If unsuccessfully, install it
- % you can localize where the git install, usually:
- % windows: C:\Program Files\Git\bin\git.exe
- % macOS or Linux: /usr/bin/git
  
# START configure: clone all the files in your project to local 
- `cd /path/to/your/matlab_project`
- `!git init`
- `!git config --global user.name <Your Name>`
- `!git config --global user.email <you@example.com>`
- `!git add . `% Add all the files
- `!git commit -m "Initial commit" `

# create a remote repository on GitHub
- create a new repository, don't initialize it, choose "code", choose SSH key
- you need to configure an SSH key
- then you get the URL of your remote repository: <git@github.com: user name/repository name.git>, you copy it
- back to the Matlab command window:
- `!git remote add origin <remote-repository-URL>`
- `!git push -u origin main` % push all the local files to your GitHub repository
  
# Finishing the START configuration
# difference between git status and git checkout
- `!git status`
- Purpose: The git status command is used to display the state of the working directory and the staging area. It shows you which changes have been staged, which haven't, and which files aren't being tracked by Git. It doesn't modify the repository in any way; it's purely informational.
- Common Use: It's often used to get a summary of what's going on in your repository before you commit changes. You can see if you need to add more files before committing or if there are untracked files you need to decide on.
- `!git checkout`
- Purpose: The git checkout command is used to switch between branches or restore working tree files. It can be used to update the files in the working directory to match the version stored in a branch or commit, which effectively allows you to view your project at a different point in history.
- Common Use: Switching Branches: To switch from one branch to another, updating the working directory to match the target branch.
- Restoring Files: To discard changes in the working directory, reverting back to the last committed version of a file:
- `!git checkout -- <file>`

# Use "push" and "pull" to keep updated between the GitHub repository and Matlab codes
Everytime after making changes to the codes, please follow the:
- `cd(pwd)` % navigate to the MATLAB file
- `!git status` % check where are the changes or using: system('git status');
- `!git add .` % determine what kind of changes will be selected to submit, select all changes
- `!git add path/to/your/file` % select specified files
- `!git commit -m "describe the changes"` % commit = submit 
- `!git push` % push to the GitHub repository
Every time Before coding, update the changes from the GitHub repository
- `!git pull origin main` % if rebase
- `!git pull` % if merge
- `!git config --get pull.rebase` % to check rebase or merge, true = rebase
% always rebase: !git config --global pull.rebase true
% always merge: !git config --global pull.rebase false
% For private projects: choose to rebase, more clean
If you need to access the former version of the codes
- `cd(pwd)`
- `!git log` % access the commit hash of all versions, pick the commit hash of the specified/target version 
- `!git checkout a1b2c3d` % switch to the target version (a1b2c3d), reading/editing, if not commit to a new branch, the changes will lost 
- `!git checkout main` % return to the new version
If I need to store the editing of the old specified version
- `cd(pwd)`
- `!git log` % % access the commit hash of all versions, pick the "commit hash: a1b2c3d" of the specified/target version 
- `!git checkout a1b2c3d` % switch to the target version % you can read and editing the target version
- `!git checkout -b new-feature-from-old-version` % after editing, stored the old version as a new branch
- `!git add .`
- `!git commit -m "Added new feature based on old version"` % select and commit
- `!git push -u origin new-feature-from-old-version`

# create a new branch for different scenarios
- `!git checkout main`
- `!git checkout -b new-scenario` % create
- `!git add .`
- `!git commit -m "Added new scenario"`
- `!git push origin new-scenario`

# change the name of the new branch
- `!git branch`
- `!git checkout new-scenario`
- `!git branch -m new-scenario-name`
- `!git push origin -u new-scenario-name`
- `!git push origin --delete new-scenario`

# How to switch between different branch, read or edit and commit
- `!git branch`
- `!git checkout main`
- `!git add .`
- `!git commit -m "commit message"`
- `!git push`

- `!git checkout new-scenario`
- `!git add . `
- `!git commit -m "commit message"`
- `!git push`

# I already have a new branch, and I made some changes in the main, I need to update the changes in the main to the new branch, 
- `!git checkout new-scenario-extend-state`
- `!git merge main`
- `!git add .`
- `!git commit -m "Merge changes from main"`
- `!git push origin new-scenario-extend-state` % here can happen conflicts, git will adapt the conflicts first, if not work, need to be solved by hand

# delete a branch： both at local and remote
- `!git branch -d new-scenario-extend-state`
- `!git push origin --delete new-scenario-extend-state`

# configure the '.gitignore' file
- method 1: go to the local repository folder： Command + Shift + . temporarily check the hidden files, open the .gitignore file, edit, save, and open the terminal window to commit the changes in the .gitignore file.
- method 2: Matlab command window：`edit .gitignore` 即可。

# ParticleFilter-simple-case
- Project description:
This is a simple case using a particle filter, the language is Matlab. 
Estimate the location of a single vehicle with constant speed 
- Purpose: 
this repository is used for source control.
- The repository contains: 
File of functions: 
generating states and measurements; 
probability distribution functions; 
resampling algorithms; 
particle filter function;
MainSripts;
- 


