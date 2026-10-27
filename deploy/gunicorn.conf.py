"""Production WSGI settings for the rover dashboard.

gunicorn replaces the Flask development server and — crucially — is one of
the few servers simple_websocket can upgrade to WebSocket on: it injects
`gunicorn.socket` into the WSGI environ (waitress does not, which is why the
otherwise-obvious choice was unusable with this app's flask-sock control
channel).

One worker only: the app owns process-wide state (background loops, camera,
pose tracker, compass fusion).  Threads carry the WebSocket clients, so
`timeout = 0` is mandatory — a sync worker would kill any request running
longer than 30 s, and a WebSocket runs forever (the dashboard pings every 2 s,
the server drops silent clients itself after 10 s).
"""

bind = "0.0.0.0:5000"
workers = 1
threads = 8
timeout = 0
graceful_timeout = 15
keepalive = 5
accesslog = "-"
errorlog = "-"
