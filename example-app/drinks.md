---
description: Models and migrations
---

# Drinks

#### models.py(APP)

{% code lineNumbers="true" %}
```python
from django.db import models

# Create your models here.
class Drinks(models.Model):
    drink = models.CharField(max_length=200)
    price = models.IntegerField()
```
{% endcode %}

#### admin.py(APP)

{% code lineNumbers="true" %}
```python
from django.contrib import admin
from .models import Drinks

# Register your models here.
admin.site.register(Drinks)
```
{% endcode %}

#### Commands fro performing migrations

```bash
# Generate operation codes
python3 manage.py makemigrations
# Applay the changes
python3 manage.py migrate

```
