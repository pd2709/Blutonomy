# Blutonomy
DMMS AUT22 Thales Blutonomy Project

Hello team!! 

🙂 Start each day off right

Begin every day by pulling or fetching from the master to your local master.

  -> Pulling will automatically try to merge the recent commits from master and throw errors if there are any conflicts

  -> Fetching will gather the most recent commits in a branch which you can then view and decide to merge or not

  -> Once you have all the most recent updates on your master you can then merge those changes into your branch:

    git checkout <branch you want to update>

    git merge <branch name you’re merging from>

You may have to handle merge conflicts at this point. Note that files with merge conflicts are usually a different color in the sidebar.

Create a new branch

    git checkout -b <new branch name>

  This branch will start off with a copy of the branch you were on.


👮Clone the New Repository to your Local Machine(s)

  Note: Each team member will need to perform this step.

  On GitHub, navigate to the project repo:

   Press the green Clone or download button.
   Ensure that clone with HTTPS is selected. (If you setup GitHub to use SSH key and passphrase, you may select that option, but these instructions assume you are using HTTPS.)
   Copy the URL provided.

In your local terminal, from the top level folder under which you want to place your repo:

    $ git clone paste-copied-url-here

🆕 Create a new branch for each new feature

From this point, each member of the group will create a new branch for any feature they are adding to the project. Do this by entering either of these two options:

    git checkout -b branchName – This creates the branch and checks it out

or

    git checkout branchName – This checks out the branch

Be sure to always check which branch you are on using “git status” before you begin working!

☁️ Merge your branch

Once your branch is ready to be merged to master, follow these steps.

   While in your branch you will 

    git add . 

    git commit -m “message”

    git push origin <branch name> – This creates the branch remotely and pushes to that branch on GitHub

  Go to GitHub and create a new pull request

   You can compare your branch to any other branch, but you will most likely be comparing to master
   You can assign a specific person or not
   You will not be able to approve your own pull request 

   Once someone reviews the pull request. they will resolve any issues or conflicts that come up and approve the pull request to be merged into the master


for more: 
https://www.digitalcrafts.com/blog/learn-how-start-new-group-project-github (please ready this carfully)
https://medium.com/anne-kerrs-blog/using-git-and-github-for-team-collaboration-e761e7c00281 (its an important reading!)
