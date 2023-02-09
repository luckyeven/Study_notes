---
description: Create a modle in Model.py file
---

# Creating models

## models.py(APP)

{% code lineNumbers="true" %}
```python
from unicodedata import name
from django.db import models

# Create your models here.
class user(models.Model):
    name = models.CharField(max_length=30)
    address = models.CharField(max_length=100)

```
{% endcode %}

Update app in settings.py at project-level folder.

{% code lineNumbers="true" %}
```python

 Application definition

INSTALLED_APPS = [
    "myModel.apps.MymodelConfig", # add this line
    "django.contrib.admin",
    "django.contrib.auth",
    "django.contrib.contenttypes",
    "django.contrib.sessions",
    "django.contrib.messages",
    "django.contrib.staticfiles",
    "testapp",
]

```
{% endcode %}

In the terminal, type the command type:\
`python3 manage.py makemigrations`\
If succeeded, the following will be printed.

```bash
Migrations for 'myModel':
  myModel/migrations/0001_initial.py
    - Create model user
```

Then run:\
`python3 manage.py migrate`

`python3 manage.py shell`

```bash
>>> from myModel.models import user
>>> user.objects.all()
<QuerySet []>
>>> new_user = user.objects.create(name='song', address='ottawa')
>>> user.objects.all()
<QuerySet [<user: user object (1)>]>
>>> new_user = user.objects.create(name='lucky', address='ottawa')
>>> user.objects.all()
<QuerySet [<user: user object (1)>, <user: user object (2)>]>
```

We can also customize the str:

{% code lineNumbers="true" %}
```python
from unicodedata import name
from django.db import models

# Create your models here.
class user(models.Model):
    name = models.CharField(max_length=30)
    address = models.CharField(max_length=100)

    def __str__(self):
        return self.name + ' lives in ' + self.address   # customize str
```
{% endcode %}

Then enter the shell again by typing: `python3 manage.py shell`

```bash
>>> from myModel.models import user
>>> user.objects.all()
<QuerySet [<user: song lives in ottawa>, <user: lucky lives in ottawa>]>
```

