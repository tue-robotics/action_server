"""
The call to the language model, through the OpenAI SDK.

The same code serves OpenAI, Azure OpenAI and any server that speaks the OpenAI chat
completions API (LLM_PROVIDER=openai with OPENAI_BASE_URL), such as a local vLLM or Ollama.
"""
import json
import time


class BackendError(Exception):
    """The language model could not be reached, or it gave no usable answer"""


class Completion(object):
    def __init__(self, text, latency, prompt_tokens=None, completion_tokens=None):
        self.text = text
        self.latency = latency
        self.prompt_tokens = prompt_tokens
        self.completion_tokens = completion_tokens


def create_client(settings):
    """
    :param settings: Settings
    :return: openai.OpenAI or openai.AzureOpenAI
    """
    # Imported here so the action server runs without the SDK as long as the LLM planner is not used
    import openai

    if settings.provider == "azure":
        return openai.AzureOpenAI(api_key=settings.api_key,
                                  azure_endpoint=settings.azure_endpoint,
                                  api_version=settings.api_version,
                                  timeout=settings.timeout)

    return openai.OpenAI(api_key=settings.api_key, base_url=settings.base_url, timeout=settings.timeout)


class OpenAIBackend(object):
    """Asks the model for one JSON object that follows a schema"""

    def __init__(self, settings, client=None):
        self._settings = settings
        self._client = client or create_client(settings)

    def complete(self, system_prompt, messages, schema, schema_name="plan"):
        """
        :param system_prompt: instructions for the model
        :param messages: the conversation so far, as a list of {"role": ..., "content": ...}
        :param schema: JSON schema of the answer
        :param schema_name: name of the schema, shown to the model
        :return: Completion; its text is a JSON object
        :raises BackendError: when the request fails, or the model refuses or returns nothing
        """
        if self._settings.response_format == "json_schema":
            # Structured output: the service only generates JSON that follows the schema
            response_format = {"type": "json_schema",
                               "json_schema": {"name": schema_name, "strict": True, "schema": schema}}
        else:
            # For services without structured output. The schema is then only an instruction.
            response_format = {"type": "json_object"}
            system_prompt += "\n\nAnswer with one JSON object that follows this JSON schema:\n" + json.dumps(schema)

        request = {"model": self._settings.model,
                   "messages": [{"role": "system", "content": system_prompt}] + list(messages),
                   "response_format": response_format}
        # Reasoning models only accept their default temperature, so it is only sent when it is set
        if self._settings.temperature is not None:
            request["temperature"] = self._settings.temperature

        hint = ""
        if self._settings.response_format == "json_schema":
            hint = " If this model or API version has no structured output, set LLM_RESPONSE_FORMAT=json_object."
        response, latency = self._send(request, hint)

        message = response.choices[0].message
        if getattr(message, "refusal", None):
            raise BackendError("The model refused: {}".format(message.refusal))
        if not message.content:
            raise BackendError("The model returned no text (finish reason: {})".format(
                response.choices[0].finish_reason))

        return self._completion(response, latency)

    def ping(self):
        """
        The smallest request there is, to check the key, the address and the model.

        :return: Completion; that the service answered is what counts, the text may be empty
        :raises BackendError: when the request fails
        """
        response, latency = self._send({"model": self._settings.model,
                                        "messages": [{"role": "user", "content": "Reply with the word ok."}]})
        return self._completion(response, latency)

    def _send(self, request, bad_request_hint=""):
        """
        :return: (the chat completion, seconds it took)
        :raises BackendError: when the request fails
        """
        import openai

        start = time.time()
        try:
            response = self._client.chat.completions.create(**request)
        except openai.BadRequestError as e:
            raise BackendError("The request was rejected: {}.{}".format(e, bad_request_hint))
        except openai.OpenAIError as e:
            raise BackendError("{}: {}".format(type(e).__name__, e))
        return response, time.time() - start

    @staticmethod
    def _completion(response, latency):
        usage = response.usage
        return Completion(text=response.choices[0].message.content or "",
                          latency=latency,
                          prompt_tokens=usage.prompt_tokens if usage else None,
                          completion_tokens=usage.completion_tokens if usage else None)
