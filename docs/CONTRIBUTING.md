# Contribution Guidelines for low-fidelity-dynamic-models

👋 Welcome to the low-fidelity-dynamic-models repository! We're excited to have you on board and look forward to your contributions. This document outlines our guidelines for contributing to this project. By adhering to these guidelines, we can maintain a high standard of quality, ensure efficient collaboration, and help you make the most impactful contributions.


## Code Contributions

### Pull Requests

- **Branching:** All contributions should be made through a branch based on the latest version of the main branch. Once your feature or bugfix is ready, submit a pull request for review.
- **Review Process:** Each pull request requires at least one review from a project maintainer before merging. This process ensures that the new code aligns with the project's direction and standards.
- **Merging:** Once approved, the pull request will be merged into the main branch by a project maintainer.

### Code Formatting

- **.clang_format File:** In the root directory of this repository, you'll find a `.clang_format` file. Use this file to format your source code consistently. 
- **IDE Integration:** If you use Visual Studio Code (VSCode), consider using the relevant extension to automate formatting (see one [here](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format)). This ensures that your code adheres to the project's style guidelines.

### Adding Data Files

- **Git LFS:** We use Git Large File Storage (LFS) to track certain types of data files. Feel free to add `.csv` files.
- **Reference Data Only:** Only reference data files (in `.csv` format) should be added to Git LFS. Do not add files that are produced as model outputs.
- **.gitignore and Output Directory:** The `.gitignore` file is configured to ignore all files in the _output_ directories/subdirectories. Store all model output in these directories to avoid accidental commits of these files.

## Reporting Issues

- If you find a bug or have a suggestion for an improvement, please first check if it has already been reported.
- When reporting a new issue, provide a clear and detailed explanation, including steps to reproduce the issue, if applicable.

## Community Guidelines

- Be respectful and considerate in your interactions with other contributors.
- Follow the code of conduct laid out for this project.
- If you're unsure about anything, don't hesitate to ask for help.

Thank you for contributing to low-fidelity-dynamic-models. Your efforts help us build a better project for everyone 😀
