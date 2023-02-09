---
description: >-
  Provide an option of giving drink_types about different items by directly
  entering the item names in the URL
---

# TypeOfDrink

## urls.py(APP)

{% code lineNumbers="true" %}
```python
from django.urls import path
from . import views
urlpatterns = [

    path('drinks/<str:drink_name>', views.drinks, name="drink_name"),
]
```
{% endcode %}

### views.py(APP)

{% code lineNumbers="true" %}
```python
from django.shortcuts import render
from django.http import HttpResponse
# Create your views here.
def drinks(request, drink_name):
    drink_item = {
        'mocha':'coffee', 
        'tea':'beverage', 
        'lemonade':'refreshment'
    }
    choice_of_drink=drink_item[drink_name]
    return HttpResponse(f"<h2> {choice_of_drink} </h2>")
    # return HttpResponse("works")

```
{% endcode %}

## urls.py(Project)

{% code lineNumbers="true" %}
```python
"""myDjangoProject URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/4.1/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""
from django.contrib import admin
from django.urls import include,path


from myapp import views
urlpatterns = [
   
    path("admin/", admin.site.urls),
    path("menu/", include('handelerror.urls')),

]

```
{% endcode %}
