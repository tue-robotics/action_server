"""
Settings of the LLM task planner: which service to call, which model, and the keys.

Every setting is an environment variable. Variables that are not set in the process
environment are read from ``<repository>/.env/llm.env``; see ``.env/llm.env.example``
for the list. The process environment wins, so a robot can be configured without a file.
"""
import os
from pathlib import Path

#: Set this variable to read the settings from another file than ``.env/llm.env``
ENV_FILE_VARIABLE = "ACTION_SERVER_LLM_ENV_FILE"

# <repository>/action_server/src/action_server/llm/settings.py -> <repository>
_REPOSITORY = Path(__file__).resolve().parents[4]
DEFAULT_ENV_FILE = _REPOSITORY / ".env" / "llm.env"

#: openai and azure are called with the OpenAI SDK, anthropic and foundry (Claude models) with the Messages API,
#: claude_code with the Claude Code CLI and its login (for development)
PROVIDERS = ("openai", "azure", "anthropic", "foundry", "claude_code")
CLAUDE_PROVIDERS = ("anthropic", "foundry")
RESPONSE_FORMATS = ("json_schema", "json_object")

ANTHROPIC_BASE_URL = "https://api.anthropic.com"
CLAUDE_CODE_PATH = "claude"


class SettingsError(Exception):
    """A setting is missing or has a value that cannot be used"""


class Settings(object):
    def __init__(self, provider, model, api_key, base_url=None, azure_endpoint=None, api_version=None,
                 timeout=20.0, temperature=None, effort=None, response_format="json_schema", knowledge_file=None,
                 claude_code_path=CLAUDE_CODE_PATH):
        self.provider = provider
        self.model = model
        self.api_key = api_key
        self.base_url = base_url
        self.azure_endpoint = azure_endpoint
        self.api_version = api_version
        self.timeout = timeout
        self.temperature = temperature
        self.effort = effort
        self.response_format = response_format
        self.knowledge_file = knowledge_file
        self.claude_code_path = claude_code_path

    def describe(self):
        """One line for logs. Never contains the key."""
        if self.provider == "azure":
            where = self.azure_endpoint
        elif self.provider == "claude_code":
            where = self.claude_code_path
        else:
            where = self.base_url or "api.openai.com"
        if self.effort:
            where += " effort=" + self.effort
        return "provider={} model={} endpoint={} response_format={}".format(
            self.provider, self.model, where, self.response_format)


def read_env_file(path):
    """
    Read KEY=VALUE lines. Empty lines and lines starting with # are skipped, quotes around a value are removed.

    :param path: file to read; a file that does not exist gives no values
    :return: dict of the values in the file
    """
    values = {}
    path = Path(path)
    if not path.is_file():
        return values

    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        values[key.strip()] = value
    return values


def _api_root(url):
    """
    The address the Messages API lives under. The Foundry portal shows the whole address of the endpoint as
    "Target URI" (.../anthropic/v1/messages); that is accepted too.
    """
    url = url.rstrip("/")
    for ending in ("/v1/messages", "/v1"):
        if url.endswith(ending):
            url = url[:-len(ending)]
    return url


def read_values(env_file=None, environ=None):
    """
    :param env_file: file with the settings; default $ACTION_SERVER_LLM_ENV_FILE, then ``.env/llm.env``
    :param environ: process environment, default os.environ
    :return: (the values of the file, overruled by the environment; the file that was read)
    """
    environ = os.environ if environ is None else environ
    env_file = Path(env_file or environ.get(ENV_FILE_VARIABLE) or DEFAULT_ENV_FILE)

    values = read_env_file(env_file)
    values.update({key: value for key, value in environ.items() if value})
    return values, env_file


def load_settings(env_file=None, environ=None):
    """
    :param env_file: file with the settings; default $ACTION_SERVER_LLM_ENV_FILE, then ``.env/llm.env``
    :param environ: process environment, default os.environ
    :raises SettingsError: when a required setting is missing
    """
    values, env_file = read_values(env_file, environ)

    def get(name):
        return values.get(name) or None

    def require(name):
        value = get(name)
        if value is None:
            raise SettingsError("{} is not set. Set it in {} or in the environment.".format(name, env_file))
        return value

    def number(name, default):
        value = get(name)
        if value is None:
            return default
        try:
            return float(value)
        except ValueError:
            raise SettingsError("{} must be a number, got '{}'".format(name, value))

    provider = (get("LLM_PROVIDER") or "openai").lower()
    if provider not in PROVIDERS:
        raise SettingsError("LLM_PROVIDER must be one of {}, got '{}'".format(", ".join(PROVIDERS), provider))

    response_format = (get("LLM_RESPONSE_FORMAT") or "json_schema").lower()
    if response_format not in RESPONSE_FORMATS:
        raise SettingsError("LLM_RESPONSE_FORMAT must be one of {}, got '{}'".format(
            ", ".join(RESPONSE_FORMATS), response_format))

    settings = Settings(provider=provider,
                        # Claude Code has a default model; the services need one
                        model=get("LLM_MODEL") if provider == "claude_code" else require("LLM_MODEL"),
                        api_key=None,
                        timeout=number("LLM_TIMEOUT", 20.0),
                        temperature=number("LLM_TEMPERATURE", None),
                        effort=get("LLM_EFFORT"),
                        response_format=response_format,
                        knowledge_file=get("LLM_KNOWLEDGE_FILE"))

    if provider == "azure":
        settings.api_key = require("AZURE_OPENAI_API_KEY")
        settings.azure_endpoint = require("AZURE_OPENAI_ENDPOINT")
        settings.api_version = require("OPENAI_API_VERSION")
    elif provider == "anthropic":
        settings.api_key = require("ANTHROPIC_API_KEY")
        settings.base_url = _api_root(get("ANTHROPIC_BASE_URL") or ANTHROPIC_BASE_URL)
    elif provider == "foundry":
        # The names the Foundry client of the Anthropic SDK reads
        settings.api_key = require("ANTHROPIC_FOUNDRY_API_KEY")
        if get("ANTHROPIC_FOUNDRY_BASE_URL"):
            settings.base_url = _api_root(get("ANTHROPIC_FOUNDRY_BASE_URL"))
        elif get("ANTHROPIC_FOUNDRY_RESOURCE"):
            settings.base_url = "https://{}.services.ai.azure.com/anthropic".format(get("ANTHROPIC_FOUNDRY_RESOURCE"))
        else:
            raise SettingsError("Set ANTHROPIC_FOUNDRY_RESOURCE or ANTHROPIC_FOUNDRY_BASE_URL in {} or in the "
                                "environment.".format(env_file))
    elif provider == "claude_code":
        # No key: Claude Code uses its own login
        settings.claude_code_path = get("CLAUDE_CODE_PATH") or CLAUDE_CODE_PATH
    else:
        settings.api_key = require("OPENAI_API_KEY")
        settings.base_url = get("OPENAI_BASE_URL")

    return settings
