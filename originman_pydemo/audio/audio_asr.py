from http import HTTPStatus
from dashscope.audio.asr import Recognition
import dashscope
from pydub import AudioSegment
import noisereduce as nr
import numpy as np
import soundfile as sf
from concurrent.futures import ThreadPoolExecutor
import time

# 设置 API Key
dashscope.api_key = "sk-3d037ba3824c40aba70b2593523ea4d0"

# 文件路径
input_audio_file = 'prompt.wav'

# 开始计时
start_time = time.time()

# 加载音频并降低采样率
audio = AudioSegment.from_file(input_audio_file, format="wav").set_frame_rate(16000)
audio.export("temp_raw.wav", format="wav")
audio_data, sample_rate = sf.read("temp_raw.wav")
print(f"采样率: {sample_rate}, 时长: {len(audio) / 1000} 秒")

# 如果是多声道，转为单声道
if audio_data.ndim > 1:
    audio_data = np.mean(audio_data, axis=1)

# 分段处理参数
chunk_duration_ms = 2000  # 每段 2 秒
chunk_samples = int(sample_rate * (chunk_duration_ms / 1000))

# 并行降噪
def denoise_chunk(chunk):
    return nr.reduce_noise(y=chunk, sr=sample_rate)

with ThreadPoolExecutor() as executor:
    chunks = [audio_data[i:i + chunk_samples] for i in range(0, len(audio_data), chunk_samples)]
    denoised_chunks = list(executor.map(denoise_chunk, [c for c in chunks if len(c) > 0]))
    denoised_data = np.concatenate(denoised_chunks)

# 保存降噪后的音频并覆盖原始文件
sf.write("temp_denoised.wav", denoised_data, sample_rate)
converted_audio = AudioSegment.from_file("temp_denoised.wav").set_channels(1).set_frame_rate(16000)
converted_audio.export(input_audio_file, format="wav")

# 删除临时文件
import os
for temp_file in ["temp_raw.wav", "temp_denoised.wav"]:
    if os.path.exists(temp_file):
        os.remove(temp_file)

# 语音识别
recognition = Recognition(model='paraformer-realtime-v2',
                          format='wav',
                          sample_rate=16000,
                          language_hints=['zh', 'en'],
                          callback=None)

result = recognition.call(input_audio_file)
if result.status_code == HTTPStatus.OK:
    print('识别结果：')
    sentences = result.get_sentence()
    if sentences:
        for sentence in sentences:
            print(sentence['text'])
    else:
        print("未识别到任何内容")
else:
    print('Error: ', result.message)