# Simplified Github Workflow for Organizing Synthesis Student Projects

## Overall Structure

There will be a **singular** GitHub repository for the entire team, called frc-3341-synthesis-2020. The repository will initially have only the 'main' branch (formerly the 'master' branch). Students will make a fork from this repository for their own projects; there is no collaboration planned for this initial effort, so these forks will not be merged into the 'main' branch.

## Software Students

Each software team member will have a fork of this main repository on their GitHub accounts. 

```
You will first need to login to Github.com using your personal Github account. Your fork will be associated with your account.
Navigate to https://github.com/frc-team-3341/frc-3341-Synthesis-2020
Select the 'Fork' button on the upper right of the page. This will create the fork and redirect to the site for your fork, with a URL containing your account info.
```

Operating in this forked repository, students will work on the main branch for their fork.

```
From the repo site for your fork, copy the URL to clone your fork by selecting the 'Clone or download' button.
Clone your forked repo from the CLI or GithubDesktop. The CLI command is: **git clone <repo URL>**. For example: git clone https://github.com/alanfhoss/frc-3341-InfiniteRecharge.git. From Github Desktop, you can clone the fork from the menu option "File -> Clone Repository...".
why you should frequently refresh your fork AND commit your changes to your fork frequently!).

To add any changed files via the CLI: **git add <files>** OR add all changes via **git add -A**. If using Github Desktop, you will note now that modified files are showing up in the 'Changes' view. Ensure that the list is correct, and uncheck any files that you don't want to commit as part of this change.
Commit your changes locally, again from the CLI: **git commit -m "A reasonable commit message that is descriptive"**. From Github Desktop, you can specify a commit message and description in the lower left of the 'Changes' view.
Before pushing, you may want to refresh your local fork as noted above.

To push to the remote for your fork: **git push** or select the 'Push' option on the Github Desktop main view following the commit.
```
