---
description: >-
  A web application works on the principle of a request-response cycle in a
  client-server architecture, following the HTTP protocol.   Generally, a
  browser sends the request in the form of a URL.
---

# Requests and URLs

## HTTP requests

The makeup of HTTP requests:

```
GET / HTTP/1.1
HOST: example.com
Accept-Language: en
```

The most commonly used HTTP methods are:

* GET   -> Retrieve information from the given server
* POST  -> Send data to the server
* PUT  -> Update whatever currently exists on the web server
* DELETE  -> Remove the resource

## HTTP response

```
HTTP /1.1 200 OK
Date: Sat, 02 Feb 2023 12:16:23 GMT
Server: Apache
Last-Modified: Tue, 01, Jan 2021 10:34:34 GMT
ETag: "34n3kbd-34v4-va4v4f4f4"
Accept-Ranges: bytes
Content-Length: 2432
Content-Type: text/html


<html>
<body>
<p>Hello</p>
</body>
</html>
```







