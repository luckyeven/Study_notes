---
description: >-
  Get to know Django and its uses, and learn how to set up your projects and
  applications.
---

# Intro

## Setting up a Django project in VS code

Python recommends using a virtual environment to build Python applications.

Python built-in module named `venv`. For example:

<pre class="language-bash"><code class="lang-bash"><strong>// To create a new environment
</strong><strong>python -m venv &#x3C;new env name>
</strong><strong>// Activate an environment
</strong><strong>source env/bin/activate
</strong><strong>// Exit the virtual environment
</strong><strong>deactivate
</strong></code></pre>

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
python3 -m manage.py runserve
```
