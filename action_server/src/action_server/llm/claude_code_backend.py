"""
The call to a Claude model through the Claude Code CLI (LLM_PROVIDER=claude_code), for development.

It uses the login of Claude Code on this machine (``claude`` then ``/login``), so commands can be planned
without a key for Azure, Foundry or OpenAI. It is not meant for a robot: every request starts a process, which
adds a few seconds, and a subscription login is for one's own use.

This is the command, which can be run in a shell to rule out our code:

    echo "go to the kitchen" | claude -p --model haiku --system-prompt "<instructions>" \
      --json-schema '{"type": "object", ...}' --tools "" --setting-sources "" --strict-mcp-config \
      --disable-slash-commands --no-session-persistence --output-format json

Without tools, settings, MCP servers and skills Claude Code is a plain model call; with them every request
would carry tens of thousands of tokens of instructions that are meant for coding.
"""
import json
import subprocess
import tempfile
import time

from .backend import BackendError, Completion

# Turns off everything that makes Claude Code an agent, so the model only answers
PLAIN_MODEL_ARGUMENTS = ["--tools", "", "--setting-sources", "", "--strict-mcp-config", "--disable-slash-commands",
                         "--no-session-persistence", "--output-format", "json"]


class ClaudeCodeBackend(object):
    """Asks the model for one JSON object that follows a schema"""

    def __init__(self, settings, run=None):
        """
        :param settings: Settings, with provider claude_code
        :param run: function with the signature of subprocess.run, to run the CLI with; default subprocess.run
        """
        self._settings = settings
        self._run = run or subprocess.run

    def complete(self, system_prompt, messages, schema, schema_name="plan"):
        """
        :param system_prompt: instructions for the model
        :param messages: the conversation so far, as a list of {"role": ..., "content": ...}
        :param schema: JSON schema of the answer
        :param schema_name: unused; the CLI does not name the schema
        :return: Completion; its text is a JSON object
        :raises BackendError: when the CLI fails, or the model refuses or returns nothing
        """
        arguments = []
        if self._settings.response_format == "json_schema":
            # Structured output: Claude Code checks the answer against the schema
            arguments += ["--json-schema", json.dumps(schema)]
        else:
            system_prompt += "\n\nAnswer with one JSON object that follows this JSON schema:\n" + json.dumps(schema)

        answer, latency = self._send(system_prompt, _conversation(messages), arguments)

        structured = answer.get("structured_output")
        text = json.dumps(structured) if structured is not None else answer.get("result")
        if not text:
            raise BackendError("The model returned no text (stop reason: {})".format(answer.get("stop_reason")))
        return _completion(answer, text, latency)

    def ping(self):
        """
        The smallest request there is, to check that the CLI is installed and logged in, and the model.

        :return: Completion; that the model answered is what counts, the text may be empty
        :raises BackendError: when the CLI fails
        """
        answer, latency = self._send(None, "Reply with the word ok.")
        return _completion(answer, answer.get("result") or "", latency)

    def _send(self, system_prompt, prompt, arguments=()):
        """
        :param system_prompt: instructions for the model; None keeps the default of Claude Code
        :param prompt: the user's turn
        :param arguments: more arguments for the CLI
        :return: (the JSON the CLI printed, as a dict; seconds it took)
        :raises BackendError: when the CLI cannot be run, fails or prints no answer
        """
        command = [self._settings.claude_code_path, "-p"] + PLAIN_MODEL_ARGUMENTS + list(arguments)
        if self._settings.model:
            command += ["--model", self._settings.model]
        if self._settings.effort:
            command += ["--effort", self._settings.effort]
        if system_prompt is not None:
            command += ["--system-prompt", system_prompt]

        start = time.time()
        try:
            # The prompt goes in on stdin. The CLI runs in the temporary folder, because it reads the CLAUDE.md of
            # the folder it runs in and of its parents.
            process = self._run(command, input=prompt, capture_output=True, text=True,
                                timeout=self._settings.timeout, cwd=tempfile.gettempdir())
        except OSError as e:
            raise BackendError("Cannot run {}: {}. Install Claude Code or set CLAUDE_CODE_PATH.".format(
                self._settings.claude_code_path, e))
        except subprocess.TimeoutExpired:
            raise BackendError("Claude Code gave no answer within {} s (LLM_TIMEOUT)".format(self._settings.timeout))
        latency = time.time() - start

        try:
            answer = json.loads(process.stdout)
        except ValueError:
            answer = None
        if not isinstance(answer, dict):
            raise BackendError("Claude Code exited with {} without an answer: {}".format(
                process.returncode, (process.stderr or process.stdout).strip()[:500]))
        if answer.get("is_error") or process.returncode != 0:
            # For example "Not logged in · Please run /login", or an unknown model
            raise BackendError("Claude Code failed ({}): {}".format(
                answer.get("subtype"), str(answer.get("result") or answer.get("errors") or process.stderr)[:500]))
        return answer, latency


def _conversation(messages):
    """
    The CLI takes one turn of the user, so a longer conversation (a plan that was sent back with the reason it
    cannot be executed) is written out in that turn.
    """
    if len(messages) == 1:
        return messages[0]["content"]
    names = {"user": "User", "assistant": "You"}
    return "\n\n".join("{}: {}".format(names.get(m["role"], m["role"]), m["content"]) for m in messages)


def _completion(answer, text, latency):
    usage = answer.get("usage") or {}
    # Claude Code caches the instructions, so most of the prompt is counted as written to or read from the cache
    prompt_tokens = sum(usage.get(key) or 0 for key in
                        ("input_tokens", "cache_creation_input_tokens", "cache_read_input_tokens")) or None
    return Completion(text=text, latency=latency, prompt_tokens=prompt_tokens,
                      completion_tokens=usage.get("output_tokens"))
