## How to use git first time

1. Open a terminal and navigate to the folder where you want to clone the repo. to. Alternatively, right-click and select "Open in terminal".
2. Enter: `git clone https://github.com/SuneSoerensen/BachelorPro`
`

## How to use it the next time (and the following)
1. Get the newest edition from github with the command: `git pull origin`
2. Do your editing,creating new files, etc. OBS: do not just delete non-ignored files! Se section about deleting below.
3. Now, when you have edited something, use the following command to add your changes: `git commit -a -m"My commit message"`
4. Now you can push your changes with: `git push origin`

**_NOTE_**: if you forget/skip the first step, you will likely get errors when trying to push!

### About deleting files
In the file `.gitignore` you can see which files and foldes are being ignored. These are safe to delete without worry. But if you want to delete ANY other files, do the following:
- For a specific file: `git rm path/to/file/fileName.type`
- For a specific folder (and everything in it): `git rm -r path/to/folder`


