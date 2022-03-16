# Blutonomy
DMMS AUT22 Thales Blutonomy Project

Hello team!! 

:) Start each day off right

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
