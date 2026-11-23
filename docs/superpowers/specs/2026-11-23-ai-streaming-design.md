# AI assistant: replies that stream

## Why

The assistant panel shows nothing until a whole round trip is over. A cloud
model thinking for a minute, or a local model on a CPU writing a paragraph,
looks like a hang. Every provider Horizon speaks can send its reply as it is
written (server-sent events). The text should appear as it comes.

## Goals

- The model's text appears in the panel as it is written, for all four
  provider kinds.
- What is kept in the conversation, and what the tools run on, is exactly what
  a non-streamed reply gives: the existing reply parsers stay the one place
  that reads a reply's meaning (refusals, fallbacks, thinking replay, tool
  calls, stop reasons).
- A server that ignores `stream: true` and answers whole still works.

## Non-goals

- Streaming tool-call arguments into the UI (a call shows when it is whole).
- Streaming in `horizon-mcp`: it serves tools to another client and never
  calls a model.

## Anti-goals

- Partial output kept or shown as the answer when the provider says it is not
  (an Anthropic refusal part-way: the partial text is taken back).
- A reply cut off mid-stream treated as complete.
- A thinking signature lost in reassembly (the next request would be refused).

## Approach

Each adapter gets a **stream reader**: it is fed the body's bytes as they
arrive and returns the text they add; when the response ends it rebuilds the
provider's own non-streamed reply JSON from the events and hands it to the
existing parser. (Considered and rejected: parsing events straight into a
`ChatResponse`, which would duplicate every parser's rules about refusals,
fallbacks and opaque blocks.)

- Anthropic: `stream: true`; the message from `message_start`, each block from
  `content_block_start`, `text_delta` / `thinking_delta` / `signature_delta` /
  `input_json_delta` / `citations_delta` applied, `message_delta`'s `delta` and
  `usage` merged. A `fallback` block is an ordinary block. An `error` event is
  an error (overloaded 529, rate limit 429, otherwise 500).
- OpenAI and compatible servers: `stream: true` (official API also
  `stream_options.include_usage`); content, refusal and tool-call deltas
  (by `index`; a new `id` at a used index is a new call) accumulated until
  `finish_reason` or `[DONE]`.
- Gemini: `:streamGenerateContent?alt=sse`; each event is a whole
  `GenerateContentResponse`; parts appended, adjacent plain text parts merged
  (a part carrying a `thoughtSignature` is kept apart, as it came).
- A stream that ends without its end marker is an error: "the connection
  closed before the reply was complete".

The transport says whether it can stream (`HttpTransport::streams()`, false by
default) and has `sendStreaming(request, part, done)`; `part` is called only
for a success status, and `done` still gets the whole body. The session
streams when the transport can and `ProviderConfig::stream` is on (default;
a checkbox in the settings, for a server that streams badly).

The observer gets `onAssistantTextPart(text)` as text arrives, then
`onAssistantText(whole)` as before, or `onAssistantTextWithdrawn()` on a
refusal. The panel shows the parts as plain text in a live entry and replaces
it with the whole reply rendered as Markdown at the end.

## Left to the builder

Names, file layout, the SSE line splitter's details (CR, LF and CRLF line ends
across chunk boundaries; comments; multi-line `data`).
