Author: `couchcamote`  
*Mar 31 Updated on Jun 16, 2020*   
#git #branching #name #convention  

Working for a big company with projects that could scale from a one-man team, then suddenly to a 20 developers team, having a manageable code repository is a need. Most Proof of Concept projects start with a repository with all changes being applied directly to the master branch. Elevating one into a proper big scale repository is a common path being taken by new developers when their small-scale project is suddenly noticed.

There're two branches categories:

## Code Flow Branches:

These branches which we expect to be permanently available on the repository, follow the flow of code changes starting from development until production.

`Development (dev)`:

    All new features and bug fixes should be brought to the development branch. Resolving developer code conflicts should be done as early as possible here.

`QA/Test (test)`:

    Contains all codes ready for QA testing.

`Staging (staging , Optional)`:

    It contains tested features that the stakeholders wanted to be available either for a demo or a proposal before elevating into production. Decisions are made here if a feature should finally  be brought to the production code.

`Master (master)`:

    The production branch, if the repository is published, this is the default branch being presented.

Except for Hotfixes, we want our codes to follow a one-way merge starting from **development > test > staging > production**.

## Temporary Branches:

As the name implies, these are disposable branches that can be created and deleted by need of the developer or deployer.

`Feature:`

    Any code changes for a new module or use case should be done on a feature branch. This branch is created based on the current development branch. When all changes are Done, a Pull Request/Merge Request is needed to put all of these to the development branch.

    Examples:
        feature/integrate-swagger
        feature/JIRA-1234
        feature/JIRA-1234_support-dark-theme

*It is recommended to use all lower caps letters and hyphen (-) to separate words unless it is a specific item name or ID. Underscore (_) can be used to separate the ID and description.*

`Bug Fix`:

    If the code changes made from the feature branch were rejected after a release, sprint or demo, any necessary fixes after that should be done on the bugfix branch.

    Examples:
        bugfix/more-gray-shades
        bugfix/JIRA-1444_gray-on-blur-fix

`Hot Fix`:

    If there is a need to fix a blocker, do a temporary patch, apply a critical framework or configuration change that should be handled immediately, it should be created as a Hotfix. It does not follow the scheduled integration of code and could be merged directly to the production branch, then on the development branch later.

    Examples:
        hotfix/disable-endpoint-zero-day-exploit
        hotfix/increase-scaling-threshold

`Experimental`:

    Any new feature or idea that is not part of a release or a sprint. A branch for playing around.

    Examples:
        experimental/dark-theme-support

`Build`:

    A branch specifically for creating specific build artifacts or for doing code coverage runs.

    Examples:
        build/jacoco-metric

`Release`:

    A branch for tagging a specific release version

    Examples:
        release/myapp-1.01.123

    Git also supports tagging a specific commit history of the repository. A release branch is used if there is a need to make the code available for checkout or use.

`Merging`:

    A temporary branch for resolving merge conflicts, usually between the latest development and a feature or Hotfix branch. This can also be used if two branches of a feature being worked on by multiple developers need to be merged, verified and finalized.

    Examples:
        merge/dev_lombok-refactoring
        merge/combined-device-support

This is the convention for Kiwibot/Kronos. It could be a better approach which could improve upon these.

Source: https://dev.to/couchcamote/git-branching-name-convention-cch  