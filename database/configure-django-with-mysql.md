---
description: Django uses a default database but we can change it.
---

# Configure Django with MySQL

Edit setting.py in the project folder. Remember, never write any personal password in the code itself.

{% code lineNumbers="true" %}
```python

 Database
# https://docs.djangoproject.com/en/4.1/ref/settings/#databases

# DATABASES = {
#     "default": {
#         "ENGINE": "django.db.backends.sqlite3",
#         "NAME": BASE_DIR / "db.sqlite3",
#     }
# }

DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.mysql',
        'OPTIONS': {
            'read_default_file': '<path>/con.cnf'
        },

    }
}

```
{% endcode %}

An example of .conf file:

{% code lineNumbers="true" %}
```vim
[client]
database = <your database name>
user = <user name to login to database>
password = <password>
default-character-set = utf8
host = 127.0.0.1
port = 3306
```
{% endcode %}

