---
description: >-
  Study note about Django and its uses, and learn how to set up your projects
  and applications.
---

# Introduction

## Setting up a Django project in VS code

Python recommends using a virtual environment to build Python applications.

Python built-in module named `venv`. For example:

```bash
// To create a new environment
python -m venv <name>
// Activate an environment
source env/bin/activate
// Exit the virtual environment
deactivate
```

## Creating a project



Open a new terminal in VS code and create a new directory for working space.

```bash
mkdir DjangoProject
```

Then enter inside the directory using cd command

```bash
cd DjangoProject
```

Next, we need to set up the virtual environment. For example, we can create and activate a new environment named `django` with using the following:

```bash
// create a new environment
python3 -m venv django
// activate the environment
source django/bin/activate
// to exit use
deactivate
```

Now, it's ready to create a new project

To do this, use Django built-in command line tools.

```bash
django-admin startproject <project name>
```

Once, the project is created, lunch the server by:

```bash
python3 manage.py runserver
```
