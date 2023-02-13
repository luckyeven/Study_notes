---
description: Single, definitive source of information about data.
---

# Model

Generally, each model maps to a single database table.

Each model is a subclass of `django.db. models.Model` .

Each attribute represents database fields.

Instead of writing a custom query for adding and retrieving database records, a model can be used.

Django also provides an access API to access the database with Python code.&#x20;

## Create

To create a table in a database using SQL;

```sql
CREATE TABLE user (
    "id" serial NOT NULL PRIMARY KEY,
    "name" varchar(30) NOT NULL
);
```

To do this using a model in Django framework, define a model class;

{% code lineNumbers="true" %}
```python
from django.db import models

class User(models.Modle):
    name = models.CharField(max_length=30)
```
{% endcode %}

`id` as primary key is not specified in the class, because Django automatically adds it.

## Insert

#### SQL

```sql
INSERT INTO user(id, name)
VALUES (1, "Luckyeven");
```

Django

{% code lineNumbers="true" %}
```python
new_user = User(id=1, "Luckyenve")
new_user.save()
```
{% endcode %}

## Read

#### SQL

```sql
SELECT * FROM user WHERE id=1;
```

#### Django

{% code lineNumbers="true" %}
```python
user = User.objects.get(id=1)
```
{% endcode %}

## Update

#### SQL

```sql
UPDATE user
SET name = "song"
WHERE id = 1;
```

#### Django

{% code lineNumbers="true" %}
```python
user = User.objects.get(id=1)
user.name = "song"
user.save()
```
{% endcode %}

## Delete

#### SQL

```sql
DELETE FROM user WHERE id = 1;
```

#### Delete

{% code lineNumbers="true" %}
```python
User.objects.filter(id=1).delete()
```
{% endcode %}

## One-to-One relationship

Example: A college can have only one principal, as one person can be a principal of only one college.

{% code lineNumbers="true" %}
```python
# college model
class college(Model): 
    CollegeID = models.IntegerField(primary_key = True) 
    name = models.CharField(max_length=50) 
    strength = models.IntegerField() 
    website=models.URLField() 
```
{% endcode %}

`CollegeID` field is the foreign key. The `on_delete` specifies the behaviour in case the associated object in the primary model is deleted. The values can be:

* CASCADE: Deletes the object containing the `ForeignKey`
* PROTECT: Prevent deletion of the referenced object.
* RESTRICT: Prevent deletion of the referenced object by raising `RestrictedError`

{% code lineNumbers="true" %}
```python
# Principal model
class Principal(models.Model): 
    CollegeID = models.OneToOneField( 
                College, 
                on_delete=models.CASCADE 
                ) 
    Qualification = models.CharField(max_length=50) 
    email = models.EmailField(max_length=50) 

```
{% endcode %}

More relations can be found here:

{% hint style="info" %}
[https://docs.djangoproject.com/en/4.1/topics/db/models/](https://docs.djangoproject.com/en/4.1/topics/db/models/)
{% endhint %}

