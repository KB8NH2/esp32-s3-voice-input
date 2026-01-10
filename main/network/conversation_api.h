#pragma once

typedef void (*conversation_reply_callback_t)(const char *reply_text);

void conversation_init(const char *url);
void conversation_set_callback(conversation_reply_callback_t cb);
void conversation_send(const char *user_text);