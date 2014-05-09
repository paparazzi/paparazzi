choice=0

while [ $choice -ne 4 ]
do

clear

echo ""; echo ""; echo "";
echo "##################### GIT ##################################################"
echo "# What do you want to do?                                                  #"
echo "#                                                                          #"
echo "# 1) Configure this machine (username, color, mergetools, ...              #"
echo "# 2) I want to make a backup of what I have now                            #"
echo "# 3) I want to try to add some code changes but I'm not sure if it will    #"
echo "#    be good and might take a while                                        #"
echo "# 4) I want to Remeber this particular version or restore a saved version  #"
echo "# 5) Quit                                                                  #"
echo "#                                                                          #"
echo "############################################################################"

echo ""
echo -n "                Choice: "
read choice

case "$choice" in
 1 ) 
	echo -n "User Name: "; 
	read name;
	echo -n "Email: ";
	read email; 
	echo $name $email;
	git config --global core.autocrlf input
	git config --global user.name "$name" 
	git config --global user.email $email
	git config --global color.ui auto     # colors for all
	git config --global alias.st status   # make `git st` work
	git config --global alias.co checkout # make `git co` work
	git config --global alias.ci commit   # make `git ci` work
	git config --global alias.br branch   # make `git br` work
	git config --global alias.up "pull --rebase"   # make `git up` work similar to svn up
	sudo apt-get update	
	sudo apt-get install kdiff3
	# git configure mergetool kdiff3
	echo ""; echo -e "\033[1mHere are the results\033[0m:";
	git config --list
	exit
	;;
 2 )  
	clear;
	echo "";echo "This is the list of all the branches you have with a * next to the active"; echo "---------------";
	git branch -v
	echo ""; echo -e "\033[1mgit branch -v\033[0m: Remember which branch you are working on! If you are on a branch then you need to push to that branch at the end (enter to continue)"
	read test
	echo "";echo "This is the list of all your remotes"; echo "---------------";
	git remote -v
	echo ""; echo -e "\033[1mgit remote -v\033[0m: This is the list of configured remote servers with their URL. Remember to which server you want to push. If the server is called \033[1morigin\033[0m then you can simply push with no arguments (enter to continue)"
	read test
	echo "";
	git status
	echo ""; echo -e "\033[1mgit status\033[0m: What has changed? Are there new files? If yes you need to 'git add FILENAME' them first (enter to continue)"
	read test 
	echo ""; 
	echo "Make a Backup: "; echo "---------------------------------" 
	echo -e "-Add untracked files or select just a few of the many modified files: \033[1mgit add FILENAME\033[0m"
	echo -e "-Remove the files that you so not need: \033[1mgit rm FILENAME\033[0m ( or \033[1mrm FILENAME\033[0m if they were never added to git)"
	echo -e "-Read all the code changes that you are about to backup: \033[1mgit diff\033[0m (use 'q' to quit)"
	echo -e "-Final check all is OK: \033[1mgit status\033[0m"
	echo -e "-Make a LOCAL backup: \033[1mgit commit -m \"Message for the LogFile\"\033[0m "
	echo -e "-Make a LOCAL backup of all modified files without needing to add them: \033[1mgit commit -a -m \"Message for the LogFile\"\033[0m "
	echo "Send to github server: "; echo "---------------------------------" 
	echo -e "-Send to github for real backup: \033[1mgit push REMOTENAME BRANCHNAME\033[0m (if REMOTENAME=origin and BRANCH=master then you can simply do \033[1mgit push\033[0m])"
	echo "Cleanup: "; echo "---------------------------------" 
	echo -e "-Clean up all local uncommitted files after you committed everything you wanted to save: \033[1mgit clean -f \033[0m[warning: this steps actually deletes stuff forever: check with git status and git diff that you really want to loose all the changes]"
	echo -e "-Revert all modified files to the version of your last commit: \033[1mgit checkout . \033[0m[warning: this steps actually deletes stuff forever: check with git status and git diff that you really want to loose all the changes]"
	# echo "Warning: following files will be deleted while no backup exists: this means you loose these forever"
	exit;
	;;
 3 )
	echo ""; echo -n "Please enter a name of a branch to try your new fancy idea: "; read branch;
	echo ""; echo "Next steps: "; 
	echo -e "-make a branch to safely try new code that is VERY easy to integrate back in your main code: \033[1mgit checkout -b $branch\033[0m"
	echo -e "-change your mind and detele it again?: \033[1mgit checkout master; git branch -d $branch\033[0m"
	echo -e "-use the master code again?: \033[1mgit checkout master\033[0m   (check how fast git does this!)"
	echo -e "-continue the work on the branch again?: \033[1mgit checkout $branch\033[0m"
	echo -e "-I want to see the list of changes compared to my master: \033[1mgit diff master $branch\033[0m (use 'q' to exit)"
	echo -e "-Ok, this code is good. I want it in my master now: \033[1mgit checkout master; git merge $branch; git branch -d $branch\033[0m"
	exit;
	;;
 4 ) 	
	echo ""; echo -e "Here is the list of all saved versions [\033[1mgit tag\033[0m]: \033[1m"; git tag;
	echo -e "\033[0m"; 
	echo -e "-Make a new TAG: \033[1mgit tag -f TAGNAME\033[0m (=save an easy link to this revision) (-f = overwrite existing with given name)"
	echo -e "-Send it to github: \033[1mgit push REMOTE_NAME --tags\033[0m (find your available REMOTE_NAMEs using  \033[1mgit remote\033[0m )"
	echo -e "-Download all tags from github: \033[1mgit fetch REMOTE_NAME\033[0m "
	echo -e "-Now use one of your tags: \033[1mgit checkout TAG_NAME\033[0m (find the available TAG_NAMEs using  \033[1mgit tag\033[0m ) (note that after this command you will be in detached head state which means that you are using a older revision and you can not commit changes here. If you want to make changes you have to make a branch from your tag)"
        exit;
	;;
 5 ) clear; exit 1 ;;
esac

done

exit

# echo "0) Download paparazzi code (otherwise you do not have this file)"
# download paparazzi:
# git clone


echo "2) I want to connect my local github directory to a new github.com directory"

# do you already have the URL of this server?
# user@github.com = Write access  github.com/account = read only
git remote -v
# add it to your list of servers
git remote paparazzi URL
# try the code on this server
git checkout paparazzi dev

echo "3) I want to download and use (=merge) all the updates from paparazzi master"

# Make sure your tree is clean and all is committed
git status
# Clean ? Download + Merge
git pull paparazzi master
# Or manually download + merge without commit to try first
  git fetch paparazzi
  git merge paparazzi master -n
# If things are not OK:
git mergetool
git commit -m "Merged XXX into YYY"


git tag "v1.3"
git push --tags

echo "6) I want to undo the changes I made to a file and get the version as it was during the last commit"

echo "WARNING: LOOSE ALL CHANGES (irriversable)"
# 1 file
git checkout FILE
# all changes?
git checkout .

echo "7) I just made a commit that I didn't want"

echo "WARNING: This is very bad: you will erase an existing commit = remove any evidence this commit was ever made: Are you SURE that you are not better off reverting the changes instead of removing a commit on the server?"

# 
git reset hard

echo "8) My working tree is a mess... revert all to the last commit"

git rm -r --cached .'

