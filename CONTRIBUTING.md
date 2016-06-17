# How to contribute

Third-party patches are essential for keeping Paparazzi great.
We want to keep it as easy as possible to contribute changes that
get things working in your use-case. There are a few guidelines that we
need contributors to follow so that we can have a chance of keeping on
top of things.

## Getting Started

* Make sure you have a [GitHub account](https://github.com/signup/free)
* Submit a ticket for your issue, assuming one does not already exist.
  * Clearly describe the issue including steps to reproduce when it is a bug.
  * Make sure you fill in the earliest version that you know has the issue.
* Fork the repository on GitHub

## Making Changes

* Create a topic branch from where you want to base your work.
  * This is usually the master branch.
  * Only target release branches if you are certain your fix must be on that
    branch.
  * To quickly create a topic branch based on master; `git branch
    my_contribution master` then checkout the new branch with `git
    checkout my_contribution`.  Please avoid working directly on the
    `master` branch.
* Make commits of logical units with descriptive commit messages and note the corresponding issue (which GitHub will autolink), e.g.:
```
[modules] air_data: fix altitude calculation

- fix altitude calculation from differential pressure
- take geoid separation into account when calculating QNH

Fixes issue #123
```
* Check for unnecessary whitespace with `git diff --check` before committing.

## Style

* C/C++ code: Two spaces, no tabs
* Use [Doxygen](http://www.doxygen.org) comments
* Python: Four spaces, no tabs, see [PEP8](http://www.python.org/dev/peps/pep-0008)
* No trailing whitespace. Blank lines should not have any space.

## Submitting Changes

* Push your changes to a topic branch in your fork of the repository.
* Submit a [pull request](https://github.com/paparazzi/paparazzi/compare/) to the repository in the paparazzi organization.

# Additional Resources

* [More information on contributing](http://wiki.paparazziuav.org/wiki/Contributing)
* [Coding style guidelines](http://docs.paparazziuav.org/latest/styleguide.html)
* [Issue tracker](https://github.com/paparazzi/paparazzi/issues)
* [General GitHub documentation](http://help.github.com/)
* [GitHub pull request documentation](http://help.github.com/send-pull-requests/)
* [![Gitter chat](https://badges.gitter.im/paparazzi/discuss.svg)](https://gitter.im/paparazzi/discuss)
