# Contributing to ROS RealSense&trade; Project

Third-party patches and contributions are welcome. We do require
you read and agree to the Contribution License Agreement below.

## Contribution License Agreement

The Intel&reg; RealSense&trade; ROS Project is developed and distributed under
a BSD license as noted in [camera/licenses/License.txt](../camera/licenses/License.txt).

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
have the right to submit it under the open source license
indicated in the file; or

(b) The contribution is based upon previous work that, to the best
of my knowledge, is covered under an appropriate open source
license and I have the right under that license to submit that
work with modifications, whether created in whole or in part
by me, under the same open source license (unless I am
permitted to submit under a different license), as indicated
in the file; or

(c) The contribution was provided directly to me by some other
person who certified (a), (b) or (c) and I have not modified
it.

(d) I understand and agree that this project and the contribution
are public and that a record of the contribution (including all
personal information I submit with it, including my sign-off) is
maintained indefinitely and may be redistributed consistent with
this project or the open source license(s) involved.

# Contribution Prerequisites

* Create a [GitHub account](https://github.com/join) if you do not already have one.
* Search the existing [GitHub Issues](../../../issues)
to see if your contribution is already documented.
  * Submit a [new GitHub Issue](../../../issues/new) if one does not exist.
    * Complete the provided template with your system configuration.
    * For Bug fixes, include directions on how to reproduce the problem.
    * For new functionality or features, please describe in detail the requirements.
* [Create a Fork](../../../fork) of the repository on GitHub for your contribution.

# Submitting Changes

* Create a Topic branch for your work.
  * Base the topic branch on the ROS development branch `<ros release>-devel`.
* Ensure changes follow the [ROS coding Style Guide](http://wiki.ros.org/StyleGuide)
* Each commit needs to be functional/compile by itself.
* Follow [Git Commit Guidelines](https://git-scm.com/book/ch5-2.html#Commit-Guidelines)
regarding commit formatting.
  * Check for unnecessary whitespace with `git diff --check` before committing.
* Rebase commits to squash where appropriate.
* Verify the Test cases found in the package 'test' directory are passing.
  * Changes with new functionality should include new test cases.
* Submit a [Pull Request](../../../compare) to the repository.
  * Ensure that it is flagged as "Able to merge", if not you may need to rebase your Fork.
  * List the Issues fixed by the Pull Request.

### Monitor Your Pull Request

* The maintainers attempt to review all Pull Requests in a timely manner; worse case
is once per week.
* If maintainers request changes or additional information for your Pull Request, the
expectation is the submitter replies within two weeks.
* Pull Requests may be closed without being merged if there is no submitter response after 3+ weeks.

# Contribution Resources

* [Create a GitHub account](https://github.com/join)
* [GitHub Pull Request documentation](https://help.github.com/articles/using-pull-requests)
* [ROS coding Style Guide](http://wiki.ros.org/StyleGuide)
* [Git Commit Guidelines](https://git-scm.com/book/ch5-2.html#Commit-Guidelines)
