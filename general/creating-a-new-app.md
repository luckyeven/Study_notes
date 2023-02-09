---
description: >-
  An app is responsible for performing one single task out of the many involved
  in the complete web application, represented by the Django project.   It
  implies that a project comprises many independent
---

# Creating a new app

When a Django project is created with the startproject command, it creates a container folder. Django puts a manage.py script and the project package folder in the outer folder.

The startapp command option of the manage.py script creates a default folder structure for the app of that name.

Here’s how to create an `newapp` in the Djangoproject folder.

```bash
python3 manage.py startapp newapp
```

After created newapp, a newapp folder is created inside the root folder and contains a few Python files.

#### views.py

A view is called when Django's URL dispatcher identifies the client's request URL and matches it with a RUL pattern defined in the `urls.py` file

We can add some function in it, for example:

{% code lineNumbers="true" %}
```python
from django.http import HttpResponse 
def greeting(request): 
    return HttpResponse("Hello, world. ")
```
{% endcode %}

#### urls.py

A file named urls.py defines the RUL patterns for the project. If the file does not exist we need to create one.

We need to provide the URL routing mechanism for the app.

Type and Save the following  `urls.py` in the newapp folder.

{% code lineNumbers="true" %}
```python
from django.urls import path 
from . import views 

urlpatterns = [ 
    path('', views.greeting, name='greeting'), 
] 
```
{% endcode %}

Next, we need to update the urlpatterns list in the project folder's `urls.py` to include the app's URL configurations.

{% code lineNumbers="true" %}
```python
from django.contrib import admin 
from django.urls import include, path 

urlpatterns = [ 
    path('greeting/', include('newapp.urls')), # new line
    path('admin/', admin.site.urls), 
] 
```
{% endcode %}

#### settings.py

Lastly, update the list of INSTALLED\_APPS in the project's setting file.

Note, we can also config database settings in this file, more details refer to Django's official document.

{% code lineNumbers="true" %}
```python
INSTALLED_APPS = [ 
    'django.contrib.admin', 
    'django.contrib.auth',  
    'django.contrib.contenttypes', 
    'django.contrib.sessions', 
    'django.contrib.messages', 
    'django.contrib.staticfiles', 
    'newapp', # updated line
] 
```
{% endcode %}

Run the Django server to see newapp.

### Alternatively

Creating a new app using:

```bash
python3 -m djgango startapp <app name>
```

Edit `views.py` file in the new app folder.

{% code lineNumbers="true" %}
```python
from django.shortcuts import render

# Create your views here.
from django.http import HttpResponse

def home(request):
    return HttpResponse("Hello, World!)
```
{% endcode %}

Then update the urlpatterns in the project folder's `urls.py.`

{% code lineNumbers="true" %}
```python
from django.contrib import admin 
from django.urls import path 

from <app name> import views
urlpatterns = [ 
    path('', include('<app name>.home'), name = 'home'), # new line
    path('admin/', admin.site.urls), 
] 
```
{% endcode %}

Run the server.
