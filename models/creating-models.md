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

Update the admin.py at app level

{% code lineNumbers="true" %}
```python
# admin.py
from django.contrib import admin
from .models import user

# Register your models here.
admin.site.register(user)
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

>>> a = user.objects.get(pk=1)
>>> a.name = 'good boy'
>>> a.save()
>>> user.objects.all()
<QuerySet [<user: good boy lives in ottawa>, <user: lucky lives in ottawa>]>
```

Check the database:

```
mysql> USE django
Reading table information for completion of table and column names
You can turn off this feature to get a quicker startup with -A

Database changed
mysql> SHOW TABLES;
+----------------------------+
| Tables_in_django           |
+----------------------------+
| auth_group                 |
| auth_group_permissions     |
| auth_permission            |
| auth_user                  |
| auth_user_groups           |
| auth_user_user_permissions |
| django_admin_log           |
| django_content_type        |
| django_migrations          |
| django_session             |
| myModel_user               |
+----------------------------+
11 rows in set (0.00 sec)

mysql> SELECT * FROM myModel_user;
+----+----------+---------+
| id | name     | address |
+----+----------+---------+
|  1 | good boy | ottawa  |
|  2 | lucky    | ottawa  |
+----+----------+---------+
2 rows in set (0.00 sec)
```

