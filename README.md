# Welcome to FRC Dev Resources!

This is team 5401's resource library for commonly used Subsystems, Commands, and CommandGroups with a basic outline of the FRC WPILib project file structure.
In this Read Me are a few tidbits of information, useful to keep in mind while coding.

## Wonky Code

With the stress and rushed atmosphere while creating code during the season, everyone creates wonky or unclean code. 
This is an inevitable byproduct of the heat of the build season, but it's easier for changes down the line if you try to get our code as clean as possible.
If you have extra time during the season, or even after build season is over, look back at your code to see if there are any improvements that could be made, so that other programmers can easily use your code as a starting point.
Solid code makes development and improvement much easier.

## Comments

Another way to make sure your code is understood by other programmers is by commenting on your code.
Commenting is very important so that all programmers can be on the same page when working on a project.
Your commented code can be used as an example to help new programmers make a correlation with descriptions to syntax.
Make sure you comment on your code!!!

## Branching

When adding more to the code, it's recommended that you create a new branch. 
The intention of your new branch should have a name to reflect it.
You can create a new branch with: (**replace `<my-branch>` with the name of your branch**)

```bash
git checkout -b <my-branch>
```

## Merging Finished Branches

Before merging your branch, you should run tests with it to make sure it is fully functional with no errors.
Testing your changes on the robot is the best way to test your code.
Then you can send a pull request to the main branch, and once everything is checked by the programming lead, they will approve the merge.
