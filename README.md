# use "push" and "pull" to keep update between the github repository and matlab codes
everytime after making changes of the codes, please follow:
- cd(pwd) % navigate to the MATLAB file
- !git status % check where are the changes or using: system('git status');
- !git add . % determine what kind of changes will be selected to submit, select all changes
- !git add path/to/your/file % select specified files
- !git commit -m "描述你的更改" % commit = submit 
- !git push % push to the github repository
everyday before coding, update the changes from github repository
- !git pull origin main % if rebase
- !git pull % if merge
- !git config --get pull.rebase % to check rebase or merge, true = rebase
% always rebase: !git config --global pull.rebase true
% always merge: !git config --global pull.rebase false
% private project: choose rebase, more clean
If need to access the former version of codes
- cd(pwd)
- !git log % access the commit hash of all versions, pick the commit hash of the specified/target version 
- !git checkout a1b2c3d % switch to the target version, reading/editing, if not commit to a new branch, the changes will lost 
- !git checkout main % return to the new version
If I need to store the editing of the old specified version
- cd(pwd)
- !git log % % access the commit hash of all versions, pick the "commit hash: a1b2c3d" of the specified/target version 
- !git checkout a1b2c3d % switch to the target version % you can read and editing the target version
- !git checkout -b new-feature-from-old-version % after editing, stored the old version as a new branch
- !git add .
- !git commit -m "Added new feature based on old version" % select and commit
- !git push -u origin new-feature-from-old-version

# configure the '.gitignore' file
方法1:在本地repository文件夹：快捷键 Command + Shift + . 来临时查看隐藏文件。然后双击打开编辑，保存，使用terminal进行commit更改
方法2:在matlab command window里：edit .gitignore 即可。

# ParticleFilter-simple-case
- Project description:
This is a simple case using particle filter, the language is matlab. 
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


