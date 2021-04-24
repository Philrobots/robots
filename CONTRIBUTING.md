# Contributing to COViRondelle2021

**Contributions are welcome!**

## Code of conduct

Before contributing to the project, please read our [code of conduct](CODE_OF_CONDUCT.md).

## Task tracking

### Functionalities to implement

The functionalities to implement for each iteration, documentation and drafts for planned features are located on our Google Drive.

### Issues

We track our issues with GitHub issues. Each issue must have at least one person assigned, a date of delivery, an associated milestone and correct labels.

Milestones represent release versions. We use [Semanting Versioning 2.0.0](https://semver.org/), which is as follows : `<major>.<minor>.<patch>`.

### Project board

Issues must be placed on the [project board](https://github.com/GLO3013-E4/COViRondelle2021/projects/1). There are 5 columns in this board : 

- Maybe : Optionnal issues to improve the codebase but with no direct value to the main features
- To do : Issues that must be done to deliver the current iteration
- In progress : Self-explanatory
- Under review : Issues currently in review or waiting to be merged
- Done : Closed issues (see : Definition of done)

The person in charge of an issue is in charge of moving it across the project board.

### Bug reporting

Everything in the app is unit tested. A test not passing is a bug.

When a bug is spotted in the application, it must be reported as an issue on GitHub issues with the appropriate template. There is a `bug` label. It must be added in the To do column of the project, above all non-bugs issues, ordered by priority.

### Branches and pull requests

We use [Git Flow](https://nvie.com/posts/a-successful-git-branching-model/) to separate our branches.

We use trunk based development with `develop` as a main branch. Every PR adding a feature to the application or solving a bug must be merged into `develop`.

For each issue, there must be at least one PR (more PRs could be added if the issue is reopened). This PR must build. Also, two reviewers must approve the PR before it is merged into `develop`. Once it is merged, it will have to pass CI check on `develop`.

PR names are as following : `What is added` (ex : `Add accounts endpoint`).

The one in charge of merging the PR is the one in charge of the associated issue.

To review a PR is a lot of things. First, you must read each added line, understand them, make sur they make sense and point out if there is any way to improve it. You must then pull the branch, test the app, make sure it works in execution by running the app and testing manually. Only approve PRs that are 100% ready to merge. Otherwise, request changes explaining clearly what must be added for approval.

One exception to the two reviewer approvals rule applies : PRs marked only as `dependencies` (like PRs made by Dependabot) require only one person to checkout the branch, test every script and make sure nothing is broken when starting the application.

**Since our repo is private and we do not want to pay for an upgraded organization, the branches cannot be automatically protected. We expect developpers to respect our pull request management process.**

`main` is our production branch. Once in a while, when `develop` is perfectly stable and operationnal, we merge `develop` into `main`.

### Definition of done

A milestone is achieved once every of its issues are solved. This includes everything to add for a new release, from adding features, to solving bugs and improving performance.

Issues are closed once all described tasks are confirmed done by the reviewers, which only means that the PR is closed. This requires reviewing code style, quality, tests and actual functionality of said PR.

## Development

Further information will be placed here when the project will have started.

### Code style

We use [eslint]https://eslint.org/) for our frontend code style. It is checked pre-commit and during CI check. To format code, use `yarn lint <filename>`.

We use [pylint](https://www.pylint.org/) for our backend code style. It is checked pre-commit and during CI check. To format code, use `pylint <filename>`.

No comment should be in the source code. Some exceptions are small explanations. In those rare cases, comments are clear and tiny.

TODOs are okay, as long as they do not make it to the release. They can be used to mark where a certain issue must be done (in which case, an issue number is much appreciated). In almost all other cases, they should be removed an converted to an actual issue.

### Test driven development

Almost every single piece of code added to the application must be written using test driven development. Of course, we're in a robotics project, so we don't expect everything so much, but we still aim for it. For TDD, we follow the three basic steps : write failing tests for new feature, write basic code to get tests to pass and finally reformat newly added code. Once the new feature is correctly implemented, commit.

Unit tests must have sections Arrange-Act-Assert separated by one blank line. Set up of tests must be extracted as much as possible from unit tests. Unit test names should follow a given, when then structure (example :  `Given obstacle in direct trajectory when creating trajectory then return corrected trajectory`).

## Contributors

### Electrical engineers (GEL)

- Frédéric Paradis ([FredPara](https://github.com/FredPara))
- Félix Bergeron ([FlixBerg](https://github.com/FlixBerg))
- Gabriel Desjardins ([Gabdesj](https://github.com/Gabdesj))

### Computer engineers (GIF)

- David Roussel ([davidaroussel](https://github.com/davidaroussel)) (project lead)
- Olivier Lajoie ([labonnesauce](https://github.com/labonnesauce)) (GEL/GIF team lead)
- Dave Caron ([davecaron](https://github.com/davecaron))

### Software engineers (GLO)

- Fabien Roy ([ExiledNarwal28](https://github.com/ExiledNarwal28)) (GLO team lead)
- Philippe Vincent ([Philrobots](https://github.com/Philrobots))
- D.Beno Awoussi ([BeastBeno](https://github.com/BeastBeno))
- Maxime Miville Deschênes ([maximemvd](https://github.com/maximemvd))
- Virginie D'Astous ([vidas4](https://github.com/vidas4))
- Vincent Breault ([VinceBro](https://github.com/VinceBro))
- Benjamin Girard ([girardb](https://github.com/girardb))
