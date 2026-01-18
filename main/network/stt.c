#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "stt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <errno.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

static const char *TAG = "stt";

#define STT_SAMPLE_RATE 16000
#define DEVICE_SAMPLE_RATE 16000

// Build a 44-byte WAV header for PCM16 mono
static void build_wav_header(uint8_t *header, uint32_t pcm_data_size) {
    uint32_t file_size = pcm_data_size + 36;
    uint32_t sample_rate = STT_SAMPLE_RATE;
    uint32_t byte_rate = sample_rate * 2; // 16-bit mono
    memcpy(header, "RIFF", 4);
    header[4] = file_size & 0xff;
    header[5] = (file_size >> 8) & 0xff;
    header[6] = (file_size >> 16) & 0xff;
    header[7] = (file_size >> 24) & 0xff;

    memcpy(header + 8, "WAVE", 4);
    memcpy(header + 12, "fmt ", 4);

    header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0; // Subchunk1Size
    header[20] = 1; header[21] = 0;   // PCM
    header[22] = 1; header[23] = 0;   // Mono

    // SampleRate (little-endian)
    header[24] = sample_rate & 0xff;
    header[25] = (sample_rate >> 8) & 0xff;
    header[26] = (sample_rate >> 16) & 0xff;
    header[27] = (sample_rate >> 24) & 0xff;

    header[28] = byte_rate & 0xff;
    header[29] = (byte_rate >> 8) & 0xff;
    header[30] = (byte_rate >> 16) & 0xff;
    header[31] = (byte_rate >> 24) & 0xff;

    header[32] = 2; header[33] = 0;   // BlockAlign
    header[34] = 16; header[35] = 0;  // BitsPerSample

    memcpy(header + 36, "data", 4);
    header[40] = pcm_data_size & 0xff;
    header[41] = (pcm_data_size >> 8) & 0xff;
    header[42] = (pcm_data_size >> 16) & 0xff;
    header[43] = (pcm_data_size >> 24) & 0xff;
}

// send helper with non-blocking retry using socket options
static int socket_send_all(int sock, const void *buf, int len, int timeout_ms)
{
    int sent = 0;
    int64_t start = esp_timer_get_time();
    while (sent < len) {
        int to_send = len - sent;
        int s = send(sock, (const char *)buf + sent, to_send, 0);
        if (s > 0) {
            sent += s;
            continue;
        }
        if (s == 0) return sent;
        if (s < 0) {
            int e = errno;
            if (e == EWOULDBLOCK || e == EAGAIN) {
                int64_t now = esp_timer_get_time();
                if ((now - start) / 1000 > timeout_ms) return -1;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            ESP_LOGE(TAG, "send failed errno=%d", e);
            return -1;
        }
    }
    return sent;
}

char *stt_send_wav_multipart(const int16_t *pcm_data, size_t pcm_samples)
{
    ESP_LOGD(TAG, "stt_send_wav_multipart enter: pcm_samples=%u", (unsigned)pcm_samples);
    const char *host = "192.168.1.154";
    const char *port = "10300";
    const char *path = "/asr";
    const char *boundary = "----ESP32Boundary";

    int out_samples = pcm_samples;
    bool resample_needed = false;
    double rescale = 1.0;
    if (DEVICE_SAMPLE_RATE != STT_SAMPLE_RATE) {
        resample_needed = true;
        out_samples = (int)((int64_t)pcm_samples * STT_SAMPLE_RATE / DEVICE_SAMPLE_RATE);
        rescale = (double)(pcm_samples - 1) / (double)(out_samples - 1);
        ESP_LOGD(TAG, "Resample streaming: out_samples=%d out_bytes=%d", out_samples, out_samples * 2);
        ESP_LOGD(TAG, "stt: after resample calculation");
        ESP_LOGD(TAG, "stt: checkpoint after resample calc time=%lld", (long long)esp_timer_get_time());
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    int pcm_bytes = out_samples * 2;

    ESP_LOGD(TAG, "stt_send_wav_multipart: pcm_samples=%u out_samples=%d pcm_bytes=%d resample=%d",
             (unsigned)pcm_samples, out_samples, pcm_bytes, resample_needed);
    uint8_t wav_header[44];
    ESP_LOGD(TAG, "stt: before build_wav_header");
    build_wav_header(wav_header, pcm_bytes);
    ESP_LOGD(TAG, "stt: after build_wav_header");

    char form_header[256];
    int header_len = snprintf(form_header, sizeof(form_header),
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"audio_file\"; filename=\"audio.wav\"\r\n"
        "Content-Type: audio/wav\r\n\r\n",
        boundary);

    char form_footer[64];
    int footer_len = snprintf(form_footer, sizeof(form_footer), "\r\n--%s--\r\n", boundary);

    int total_len = header_len + 44 + pcm_bytes + footer_len;
    ESP_LOGD(TAG, "stt: total_len=%d header_len=%d footer_len=%d pcm_bytes=%d", total_len, header_len, footer_len, pcm_bytes);
    const int MAX_UPLOAD_BYTES = 256 * 1024;
    if (total_len <= 0 || total_len > MAX_UPLOAD_BYTES) {
        ESP_LOGE(TAG, "upload too large or invalid: %d bytes", total_len);
        return NULL;
    }

    // Resolve numeric IP first to avoid blocking DNS calls
    char ipstr[INET_ADDRSTRLEN] = "";
    struct sockaddr_in dest;
    int have_numeric = 0;
    struct addrinfo *res = NULL;
    if (inet_pton(AF_INET, host, &dest.sin_addr) == 1) {
        dest.sin_family = AF_INET;
        dest.sin_port = htons(atoi(port));
        strncpy(ipstr, host, sizeof(ipstr)-1);
        have_numeric = 1;
        ESP_LOGD(TAG, "Using numeric IP %s", ipstr);
    }

    int sock = -1;
    if (have_numeric) {
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    } else {
        // fallback to getaddrinfo if host isn't numeric
        struct addrinfo hints = {0}, *res = NULL;
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        int err = getaddrinfo(host, port, &hints, &res);
        if (err != 0 || !res) {
            ESP_LOGE(TAG, "getaddrinfo failed: %d", err);
            return NULL;
        }
        struct sockaddr_in *sa = (struct sockaddr_in *)res->ai_addr;
        inet_ntop(AF_INET, &(sa->sin_addr), ipstr, sizeof(ipstr));
        ESP_LOGD(TAG, "Resolved %s to %s", host, ipstr);
        sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        // copy resolved address to dest for connect
        memcpy(&dest, sa, sizeof(dest));
        // keep res so callers can free it later if needed
        // freeaddrinfo(res);
    }
    if (sock < 0) {
        ESP_LOGE(TAG, "socket failed: %d", errno);
        return NULL;
    }
    ESP_LOGD(TAG, "socket created: fd=%d", sock);

    // set send/recv timeout - increase recv timeout for Whisper processing
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    
    // Whisper can take 30+ seconds to process audio
    tv.tv_sec = 60;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Use non-blocking connect with timeout to avoid hard hangs
    int old_flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, old_flags | O_NONBLOCK);
    ESP_LOGI(TAG, "Connecting to STT server %s:%s...", ipstr, port);
    int rc = connect(sock, (struct sockaddr *)&dest, sizeof(dest));
    if (rc != 0) {
        if (errno == EINPROGRESS) {
            ESP_LOGD(TAG, "connect in progress, waiting up to 5000 ms");
            fd_set wf;
            FD_ZERO(&wf);
            FD_SET(sock, &wf);
            struct timeval tv;
            tv.tv_sec = 5;
            tv.tv_usec = 0;
            int sel = select(sock + 1, NULL, &wf, NULL, &tv);
            if (sel <= 0) {
                ESP_LOGE(TAG, "STT server connect timeout (is Whisper running on %s:%s?)", ipstr, port);
                close(sock);
                if (res) freeaddrinfo(res);
                return NULL;
            }
            int so_error = 0;
            socklen_t len = sizeof(so_error);
            getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
            if (so_error != 0) {
                ESP_LOGE(TAG, "STT server connection refused (errno=%d). Check if Whisper is running at %s:%s", so_error, ipstr, port);
                close(sock);
                if (res) freeaddrinfo(res);
                return NULL;
            }
            ESP_LOGI(TAG, "Connected to STT server successfully");
        } else {
            ESP_LOGE(TAG, "STT server connect failed: errno=%d (%s)", errno, strerror(errno));
            close(sock);
            if (res) freeaddrinfo(res);
            return NULL;
        }
    } else {
        ESP_LOGD(TAG, "connect returned immediately success");
    }
    // restore blocking mode
    fcntl(sock, F_SETFL, old_flags);
    if (res) freeaddrinfo(res);
    ESP_LOGD(TAG, "socket connected to %s:%s", ipstr, port);

    // Build and send HTTP request header (with Content-Length)
    char req_hdr[512];
    int req_len = snprintf(req_hdr, sizeof(req_hdr),
        "POST %s HTTP/1.1\r\n"
        "Host: %s:%s\r\n"
        "User-Agent: ESP32\r\n"
        "Accept: application/json\r\n"
        "Content-Type: multipart/form-data; boundary=%s\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n",
        path, host, port, boundary, total_len);

    if (socket_send_all(sock, req_hdr, req_len, 5000) != req_len) {
        ESP_LOGE(TAG, "Failed to send HTTP request header (server may have closed connection)");
        close(sock);
        return NULL;
    }
    ESP_LOGI(TAG, "Sent HTTP POST request to %s, total payload: %d bytes", path, total_len);

    // send form header
    if (socket_send_all(sock, form_header, header_len, 5000) != header_len) {
        ESP_LOGE(TAG, "Failed to send form header");
        close(sock);
        return NULL;
    }
    // send wav header
    if (socket_send_all(sock, wav_header, 44, 5000) != 44) {
        ESP_LOGE(TAG, "Failed to send WAV header");
        close(sock);
        return NULL;
    }

    // send PCM
    const int CHUNK_SAMPLES = 1024;
    if (!resample_needed) {
        const uint8_t *pcm_ptr = (const uint8_t *)pcm_data;
        int bytes_remaining = pcm_bytes;
        const int CHUNK_BYTES = 1024;
        while (bytes_remaining > 0) {
            int chunk = bytes_remaining > CHUNK_BYTES ? CHUNK_BYTES : bytes_remaining;
            // Use 60-second timeout for PCM chunks to handle slow server processing
            if (socket_send_all(sock, pcm_ptr, chunk, 60000) != chunk) {
                ESP_LOGE(TAG, "failed sending pcm chunk");
                close(sock);
                return NULL;
            }
            pcm_ptr += chunk;
            bytes_remaining -= chunk;
        }
    } else {
        int res_index = 0;
        int16_t chunk_buf[CHUNK_SAMPLES];
        while (res_index < out_samples) {
            int to_do = out_samples - res_index;
            if (to_do > CHUNK_SAMPLES) to_do = CHUNK_SAMPLES;
            for (int j = 0; j < to_do; ++j) {
                int global_j = res_index + j;
                double t = global_j * rescale;
                int i0 = (int)floor(t);
                double frac = t - i0;
                int16_t s0 = pcm_data[i0];
                int16_t s1 = (i0 + 1 < pcm_samples) ? pcm_data[i0 + 1] : pcm_data[pcm_samples - 1];
                double v = s0 * (1.0 - frac) + s1 * frac;
                chunk_buf[j] = (int16_t)lround(v);
            }
            int bytes = to_do * sizeof(int16_t);
            if (socket_send_all(sock, chunk_buf, bytes, 5000) != bytes) { ESP_LOGE(TAG, "failed sending resampled chunk"); close(sock); return NULL; }
            res_index += to_do;
        }
    }

    // send footer
    if (socket_send_all(sock, form_footer, footer_len, 5000) != footer_len) { ESP_LOGE(TAG, "failed sending footer"); close(sock); return NULL; }

    // read response
    char resp_buf[4096];
    int total = 0;
    int r;
    for (;;) {
        r = recv(sock, resp_buf + total, sizeof(resp_buf) - total - 1, 0);
        if (r > 0) {
            total += r;
            if (total >= (int)sizeof(resp_buf) - 1) break;
            continue;
        }
        if (r == 0) {
            // peer closed connection
            break;
        }
        // r < 0: error condition
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No more data available right now on non-blocking socket —
            // treat as end of response and stop reading. This can happen
            // when socket is non-blocking or due to transient conditions.
            ESP_LOGD(TAG, "recv would block (EAGAIN/EWOULDBLOCK), treating as EOF");
            break;
        }
        ESP_LOGW(TAG, "recv returned %d errno=%d", r, errno);
        break;
    }
    resp_buf[total] = '\0';
    close(sock);

    ESP_LOGD(TAG, "HTTP response received: %d bytes", total);
    if (total > 0) {
        ESP_LOGD(TAG, "Response preview (first 200 chars): %.200s", resp_buf);
    }
    // Helper: extract body and decode chunked transfer if present
    char *body = NULL;
    char *hdr_end = NULL;
    if (total > 4) hdr_end = strstr(resp_buf, "\r\n\r\n");
    int resp_header_len = hdr_end ? (int)(hdr_end - resp_buf) : 0;
    char *headers_lower = NULL;
    int is_chunked = 0;
    if (resp_header_len > 0) {
        headers_lower = malloc(resp_header_len + 1);
        if (headers_lower) {
            for (int i = 0; i < resp_header_len; ++i) headers_lower[i] = tolower((unsigned char)resp_buf[i]);
            headers_lower[resp_header_len] = '\0';
            if (strstr(headers_lower, "transfer-encoding: chunked") != NULL) is_chunked = 1;
        }
    }
    if (is_chunked && hdr_end) {
        // Decode chunked body starting after header end
        char *p = hdr_end + 4;
        int remaining = total - (int)(p - resp_buf);
        // allocate output buffer conservatively
        char *out = malloc(remaining + 1);
        if (!out) {
            free(headers_lower);
            return NULL;
        }
        int out_len = 0;
        while (remaining > 0) {
            // read chunk size line
            char *eol = memchr(p, '\r', remaining);
            if (!eol) break;
            int line_len = (int)(eol - p);
            char tmp[32] = {0};
            int copy_len = line_len < (int)sizeof(tmp)-1 ? line_len : (int)sizeof(tmp)-1;
            memcpy(tmp, p, copy_len);
            tmp[copy_len] = '\0';
            unsigned int chunk_size = (unsigned int)strtoul(tmp, NULL, 16);
            if (chunk_size == 0) break;
            // move p past CRLF
            if (remaining < line_len + 2) break;
            p += line_len + 2; remaining -= line_len + 2;
            if (remaining < (int)chunk_size) break;
            memcpy(out + out_len, p, chunk_size);
            out_len += chunk_size;
            p += chunk_size;
            remaining -= chunk_size;
            // skip trailing CRLF after chunk
            if (remaining >= 2) { p += 2; remaining -= 2; } else break;
        }
        out[out_len] = '\0';
        body = out;
    } else if (hdr_end) {
        // Not chunked: return everything after headers (may include full body)
        char *p = hdr_end + 4;
        int body_len = total - (int)(p - resp_buf);
        body = malloc(body_len + 1);
        if (body) {
            memcpy(body, p, body_len);
            body[body_len] = '\0';
        }
    } else {
        // No headers found — return whole response
        body = malloc(total + 1);
        if (body) {
            memcpy(body, resp_buf, total);
            body[total] = '\0';
        }
    }
    free(headers_lower);
    // Trim trailing CR/LF
    if (body) {
        int bl = strlen(body);
        while (bl > 0 && (body[bl-1] == '\n' || body[bl-1] == '\r')) { body[--bl] = '\0'; }
        ESP_LOGD(TAG, "Parsed body (%d bytes): %s", bl, body);
    } else {
        ESP_LOGW(TAG, "Failed to parse response body");
    }
    return body;
}

// write_all_with_timeout removed; socket-based implementation used instead