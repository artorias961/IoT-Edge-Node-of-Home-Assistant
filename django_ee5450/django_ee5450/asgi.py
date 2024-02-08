"""
ASGI config for django_ee5450 project.

It exposes the ASGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/4.2/howto/deployment/asgi/
"""

import os

from channels.auth import AuthMiddlewareStack
from channels.routing import ProtocolTypeRouter, URLRouter
from channels.security.websocket import AllowedHostsOriginValidator
from django.core.asgi import get_asgi_application

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'django_ee5450.settings')

django_asgi_http_app = get_asgi_application()

import bleak_django_test.routing

application = ProtocolTypeRouter({
    "http": django_asgi_http_app,
    "websocket": AllowedHostsOriginValidator(
            AuthMiddlewareStack(URLRouter(bleak_django_test.routing.websocket_urlpatterns))
    ),
})
