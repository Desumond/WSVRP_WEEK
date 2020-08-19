#!/usr/bin/env python
# encoding: utf8
from django.conf.urls import url

from spyne.protocol.soap import Soap11
from spyne.protocol.json import JsonDocument
from spyne.protocol.xml import XmlDocument
from spyne.server.django import DjangoView

from rpctest.core.views import MakeModeling


urlpatterns = [
	url(r'^get_solution/', DjangoView.as_view(
		services=[MakeModeling], tns='spyne.examples.django',
		in_protocol=JsonDocument(), out_protocol=JsonDocument()))
	]

#in_protocol=Soap11(validator="lxml"), out_protocol=JsonDocument()))