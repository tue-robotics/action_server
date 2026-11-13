"""
The call to a Claude model, through the Messages API.

Serves the Claude API (LLM_PROVIDER=anthropic) and Claude in Microsoft Foundry (LLM_PROVIDER=foundry): the same
API at another address. Claude in Foundry has no OpenAI-style endpoint, so the OpenAI SDK cannot reach it.

The request is made with requests instead of the Anthropic SDK because ROS Noetic runs Python 3.8. The newest
SDK release for that Python (0.72) has neither the Foundry client nor structured output. On Python 3.10 or
newer, replace this class by anthropic.Anthropic / anthropic.AnthropicFoundry.

It is this request, which can be sent with curl to check a key or a deployment (see .env/README.md):

    curl https://<resource>.services.ai.azure.com/anthropic/v1/messages \
      -H "x-api-key: <key>" -H "anthropic-version: 2026-10-21" -H "content-type: application/json" \
      -d '{"model": "<deployment>", "max_tokens": 16000, "system": "<instructions>",
           "messages": [{"role": "user", "content": "<command>"}],
           "output_config": {"format": {"type": "json_schema", "schema": {...}}}}'
"""
import json
import time

import requests

from .backend import BackendError, Completion

API_VERSION = "2026-10-21"

# Room for the reasoning the model does before it answers; a plan itself is a few hundred tokens
MAX_TOKENS = 16000

# Seconds to reach the server. The time to wait for the answer after that is the LLM_TIMEOUT setting.
CONNECT_TIMEOUT = 3.05


class ClaudeBackend(object):
    """Asks the model for one JSON object that follows a schema"""

    def __init__(self, settings, session=None):
        """
        :param settings: Settings, with provider anthropic or foundry
        :param session: requests.Session to send the requests with; default a new one
        """
        self._settings = settings
        self._url = settings.base_url + "/v1/messages"

        # A session keeps the connection to the server open, so only the first command pays for setting it up
        self._session = session or requests.Session()
        # Foundry also documents an api-key header, but our resource answers 401 to it (2026-11-13) and
        # accepts x-api-key, which is the header of the Claude API as well
        self._session.headers.update({"anthropic-version": API_VERSION, "x-api-key": settings.api_key})

    def complete(self, system_prompt, messages, schema, schema_name="plan"):
        """
        :param system_prompt: instructions for the model
        :param messages: the conversation so far, as a list of {"role": ..., "content": ...}
        :param schema: JSON schema of the answer
        :param schema_name: unused; the Messages API does not name the schema
        :return: Completion; its text is a JSON object
        :raises BackendError: when the request fails, or the model refuses or returns nothing
        """
        output_config = {}
        if self._settings.response_format == "json_schema":
            # Structured output: the model can only generate JSON that follows the schema
            output_config["format"] = {"type": "json_schema", "schema": schema}
        else:
            system_prompt += "\n\nAnswer with one JSON object that follows this JSON schema:\n" + json.dumps(schema)
        if self._settings.effort:
            output_config["effort"] = self._settings.effort

        # No temperature: this version of the API does not accept it
        body = {"model": self._settings.model, "max_tokens": MAX_TOKENS, "system": system_prompt,
                "messages": list(messages)}
        if output_config:
            body["output_config"] = output_config

        answer, latency = self._send(body)

        stop_reason = answer.get("stop_reason")
        if stop_reason == "refusal":
            raise BackendError("The model refused: {}".format(answer.get("stop_details")))
        if stop_reason == "max_tokens":
            raise BackendError("The answer was cut off after {} tokens".format(MAX_TOKENS))

        text = _text(answer)
        if not text:
            raise BackendError("The model returned no text (stop reason: {})".format(stop_reason))

        usage = answer.get("usage", {})
        return Completion(text=text, latency=latency,
                          prompt_tokens=usage.get("input_tokens"), completion_tokens=usage.get("output_tokens"))

    def ping(self):
        """
        The smallest request there is, to check the key, the address and the deployment.

        :return: Completion; that the service answered is what counts, the text may be empty
        :raises BackendError: when the request fails
        """
        answer, latency = self._send({"model": self._settings.model, "max_tokens": 64,
                                      "messages": [{"role": "user", "content": "Reply with the word ok."}]})
        usage = answer.get("usage", {})
        return Completion(text=_text(answer) or "", latency=latency,
                          prompt_tokens=usage.get("input_tokens"), completion_tokens=usage.get("output_tokens"))

    def _send(self, body):
        """
        :return: (the answer of the Messages API as a dict, seconds it took)
        :raises BackendError: when the service cannot be reached or does not answer with a message
        """
        start = time.time()
        try:
            # json= encodes the body and sets the content-type header. Without a timeout requests waits forever.
            response = self._session.post(self._url, json=body, timeout=(CONNECT_TIMEOUT, self._settings.timeout))
        except requests.RequestException as e:
            raise BackendError("{}: {}".format(type(e).__name__, e))
        latency = time.time() - start

        if response.status_code != 200:
            raise BackendError("HTTP {} from {}: {} (request id {})".format(
                response.status_code, self._url, _error_message(response), response.headers.get("request-id")))

        try:
            return response.json(), latency
        except ValueError:
            raise BackendError("The answer of {} is not JSON: {}".format(self._url, response.text[:200]))


def _text(answer):
    """The text of an answer; thinking blocks can come before it"""
    return next((block.get("text") for block in answer.get("content", []) if block.get("type") == "text"), None)


def _error_message(response):
    try:
        return response.json()["error"]["message"]
    except (ValueError, KeyError, TypeError):
        return response.text[:500]
