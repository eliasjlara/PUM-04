# AIDA

AIDA is a robot worked on by group 4 in the course TDDD96. The project is request by IDA and is currently worked on by eight people simultaneously, which is in turn, why this repo exists! The repo is used to assist in the creation of code as well as keeping track of our kandban board and other iterative processes. In this README there is a guide for use on how to use the repo, as well as some useful links. 

## How to use this repo
This is a guide on how to use this git repo, both for code development and for the iterative development, to maintain consistency and our make work easier. 

### CODE DEVELOPMENT

First and foremost, all development is supposed to happen on different branches, in fact, it is impossible to push to the main branch, trying to push to main will result in: 
```zsh
remote: error: GH006: Protected branch update failed for refs/heads/main.
remote: error: Changes must be made through a pull request.
To https://github.com/eliasjlara/PUM-04.git
 ! [remote rejected] main -> main (protected branch hook declined)
error: failed to push some refs to 'https://github.com/eliasjlara/PUM-04.git'
```
This is a safety precaution to keep bugs and non-approved code from the most important branch, the main one! To add a feature you have been coding on, follow these steps: 

**Steps on you local machine**
1. Check that you're on the main branch using `git branch`, if you're not, do `git checkout main`.
2. Pull the latest changes from main using `git pull`.
3. Create a branch with a name relating to the ticket name using `git checkout -b <good-name>`.
4. Implement your changes on this branch.
5. When ready, push your changes following standard procedure: 
   - `git status`
   - `git add <changed files you want to push>`
   - `git commit -m "Make sure to have a good message describing change"`
   - `git push`
6. If this is the first push to the branch you will get something like:
```zsh
remote: 
remote: Create a pull request for 'test-pr-template' on GitHub by visiting:
remote:      https://github.com/eliasjlara/PUM-04/pull/new/test-pr-template
remote: 
To https://github.com/eliasjlara/PUM-04.git
```
Navigate over to the link in the message, or if this isn't the first time you push to the branch, you can find the pull request in the the [Pull requests tab](https://github.com/eliasjlara/PUM-04/pulls) on the repo github page. After this, you are now ready to create a pull request!

**Steps on github.com, aka creating the pull request**








