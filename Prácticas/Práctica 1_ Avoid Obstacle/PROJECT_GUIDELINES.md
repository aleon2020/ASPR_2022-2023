# Project Guidelines

This document outlines the guidelines for the project, including roles, coding standards, documentation, and GitHub management.

## Roles

The project will be managed by four authors. Even though all members will work together to code the project, there will be two main roles: Software Testers and Software Developers.
- **Software Testter** will be responsible for testing the program and suggesting new ideas to reach more advanced objectives. They will be expected to analyze test results and identify areas of improvement, and suggest new features that could be added to the project. They will also be responsible for publishing the testing results in GitHub's Issues.
- **Software Developer** will be responsible for controlling the efficiency and effectiveness of the project, and  managing all pull requests and merges between the members. They must ensure that all code is of the highest quality and that all pull requests are reviewed thoroughly before being merged.

## Coding Standards

- The project must be maintained in English, including all documentation.
- All code must pass the format test accompanied by appropriate doxygen documentation. All functions/methods and classes must include a brief explanation, and parameters must be documented if necessary. Variables must be commented when appropriate. This will ensure that all code is easy to understand and maintain.

```c++
// This is a example class with correct documentation and indenting.

/**
  * @brief This class is in charge of making foo's
  */
class Foo {

public:

  /**
    * @brief I make foo
    * @param arg1 True - Makes the foo1; False - Makes the foo2
    */
  int makeFoo(bool arg1)
  {
    
    // Commentary if necessary. Recommendation: 1 line of comentary per step/section
    // of the function.
    DoSomething();
    DoSomething();
    DoSomething();
    
    // Doing something else...
    DoSomethingElse();
    DoSomethingElse();
  }
  
private:
// Note that it is not necessary to comment every variable.

  int TOTAL_FOO = 4;

};
```
- All code missings must use TODO's to manage incomplete sections. This will allow other members of the team to easily identify sections of the code that need further work.
```c++
// -------- BAD IMPLEMENTATION ---------
  void foo()
  {
    // This must be improved
    DoSomething();
  }
  
// -------- APPROPIATE IMPLEMENTATION ---------
  
  void foo()
  {
    // TODO: Improve this
    DoSomething();
  }
```

- All commits must follow the format: [_label_] Summary of update. There are four types of labels: **Snapshot**, **Fix**, **Release** and **Update**. Snapshot label is used for regular updates, Fix is used for bug fixes, and Release and Update labels are reserved for project managing situations. This will help other members of the team quickly identify what the commit is for.

```bash
# Example of git tree
# IMPORTANT: Labels ARE NOT TAGS. Each label is written inside the commit. The tags will be used for version distributing.

PS C:\Users\<user>\Repository> git adog

*   ed8864f (origin/main, origin/HEAD) Merge pull request #1 from <User>/<Branch>
|\
| * 4dbbf1c (HEAD -> MyBranch, origin/MyBranch) [Snapshot] Something more
| * 2e112ca [Fix] Fixed some failure
| * 2e140cf [Snapshot] Something Done
|/
* 40ce8fc (main) [Snapshot] Last main snapshot
...
* 22cfd79 Initial Commit
```
  
- All changes must be made via pull requests. Direct pushes to the main branch will be rejected. Pull requests must be made from the member's branch. A member is not allowed to modify another member's branch or approve their own pull. This will ensure that all changes made to the codebase are reviewed and approved by other members of the team before being merged.

## Decision Making

All major decisions must be unanimously agreed upon in group meetings. This will ensure that all members of the team are in agreement before making any major changes. Changes to major behaviors, targets, or even these guidelines must be accepted by at least 50% of the group. In case of a tie, the decision will always be in favor of implementation. This will ensure that decisions are made quickly and efficiently.  

## Virtual Terms Acceptance Signature

Working in this repository implies accepting the project guidelines implemented in this repository. All the members below nod to have knowledge and accept this conditions.

- [X] Javier Izquierdo
- [X] Sebastian Mayorquin 
- [X] Luis Moreno
- [X] Alberto Leon
