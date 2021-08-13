# Group Tuesday

The repository for lecture Robotics and Navigation in Medicine on Summer Semester 2021 for Group Tuesday. 
--Maintained by Cenk Taskin

## How to set up your local copy of the project? 
### 1. Create a new branch with your name
- Find the plus mark near the master branch
- You can see a list of current branches if you click the pop-up list on master.
- You have to press the plus sign and "new branch"
- Please name the branch with your name in a shortened fashion. Leave it as create from "master".

### 2. Clone the repository
I use the terminal on my Mac. I will give info regarding that, you can find this information easily on web for any configuration.
- Grab the repository link from the right side "Clone" 
- Copy the link on "Clone with HTTPS" 
- Run a new Terminal window.
- Navigate to the directory that you want to have your project located.
- Type "git clone <repository_link>"

### 2. Switch to your branch
When you are working make sure you are on your branch.
- Navigate inside the folder you just created. ("cd rnm_tuesday")
- Type "git status" to see which branch you are on. You should be on master.
- "git checkout <your_branch_name>". Now you are on your branch.

## How to work on your branch?
- Make sure you are on your branch with "git status"
- When you are done make git track your files via "git add .". The dot adds all files otherwise you need to specify files/directories one by one. If you do again a "git status" you can see they are listed as tracked files.
- Do a "git commit -m <your_message>" . It is good practice to write a simple but explanatory message.
- Finally do a "git push" to push changes in local to remote. 
- The GitLab might ask you your credentials up this point. 

- You can get the updates from the master via first "git checkout <your_branch_name>" and then "git rebase master". Although I haven't used rebase before so make sure your changes are merged with master before to not to lose them, or back them up.


