# Class-based views

## Function-based views

{% code lineNumbers="true" %}
```python
# views.py

from django.http import HttpResponse

def my_view(request):
    if request.method == 'GET':
        return HttpResponse('Result')
```
{% endcode %}

Instead of using conditional branching, Class-based views respond to HTTP requests using `class instance methods`.

Instance methods are default python methods, defined in classes that can access objects or instances of a class.

## Class-based views

{% code lineNumbers="true" %}
```python
# views.py
from django.http import HttpResponse
from django.views import View

class MyView(View):
    def get(self, request):
        return HttpResponse('Result')
    
    def post(self, request):
        return HttpResponse('Result')
```
{% endcode %}

Using class-based views can make the codes easy to understand. And also adopt the ability to have object-oriented techniques.

In Django, the functionality of class-based views can be extended using `mixins`.

## Mixins

Mixins are class-based generic views that are flexible compared to their function-based view equivalent.
