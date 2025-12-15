# API Contracts for VLA Module

## Voice-to-Action Interface

### Interface Overview
The Voice-to-Action interface handles the conversion of audio input to transcribed text commands. This interface connects audio input sources to the Whisper transcription service.

### API Endpoints

#### POST /api/vla/transcribe
Transcribes audio data using Whisper and returns the transcribed text.

**Request:**
```json
{
  "audioData": "base64 encoded audio data or file reference",
  "language": "en",
  "model": "base",
  "timeout": 30
}
Response:

json
Copy code
{
  "id": "transcription-12345",
  "transcription": "Hello world",
  "confidence": 0.98,
  "language": "en",
  "processingTime": 1200,
  "status": "success"
}
Error Response:

json
Copy code
{
  "error": "Invalid audio format",
  "code": "INVALID_REQUEST",
  "details": "Audio file must be WAV or MP3"
}
POST /api/vla/realtime-transcribe
Processes audio stream in real-time for continuous transcription.

Request:

Content-Type: application/octet-stream

Transfer-Encoding: chunked

Binary audio stream data

Response (Server-Sent Events):

kotlin
Copy code
data: {"transcription": "partial text", "isFinal": false, "timestamp": "2023-01-01T10:00:00Z"}
data: {"transcription": "complete sentence", "isFinal": true, "timestamp": "2023-01-01T10:00:05Z"}
Error Handling
Return error message if audio quality is poor

Implement timeout of 30 seconds for processing

Provide confidence scores for transcriptions

LLM Cognitive Planning Interface
POST /api/vla/plan
Request:

json
Copy code
{
  "command": "Move the robot to point A",
  "context": {
    "robotPosition": [0, 0, 0],
    "environmentMap": "map-1",
    "objectLocations": {},
    "robotCapabilities": ["navigation", "manipulation"],
    "constraints": {
      "safeZones": [[0,0,10,10]],
      "forbiddenAreas": [[5,5,6,6]]
    }
  },
  "timeout": 60
}
Response:

json
Copy code
{
  "id": "plan-12345",
  "command": "Move the robot to point A",
  "actionSequence": [
    {
      "id": "action-1",
      "type": "navigation",
      "action": "move",
      "parameters": {"destination": [1, 2, 0]},
      "dependencies": [],
      "timeout": 30
    }
  ],
  "contextUsed": {
    "robotPosition": [0, 0, 0],
    "environmentMap": "map-1"
  },
  "confidence": 0.95,
  "processingTime": 500,
  "status": "success"
}
POST /api/vla/plan/clarify
Request:

json
Copy code
{
  "command": "Move to the location",
  "context": {
    "robotPosition": [0, 0, 0],
    "environmentMap": "map-1"
  }
}
Response:

json
Copy code
{
  "clarificationNeeded": "Which specific location should the robot move to?",
  "options": ["Point A", "Point B"],
  "suggestedRephrasing": "Move to Point A or Point B"
}
ROS2 Action Execution Interface
POST /api/vla/execute
Request:

json
Copy code
{
  "actionSequence": [
    {
      "id": "action-1",
      "type": "navigation",
      "action": "move",
      "parameters": {"destination": [1,2,0]},
      "timeout": 30
    }
  ],
  "executionOptions": {
    "continueOnError": false,
    "maxRetries": 3,
    "safetyCheck": true
  }
}
Response:

json
Copy code
{
  "executionId": "exec-12345",
  "status": "executing",
  "estimatedCompletionTime": 30,
  "actionResults": [
    {
      "actionId": "action-1",
      "status": "pending",
      "result": null,
      "executionTime": null
    }
  ]
}
GET /api/vla/execute/{executionId}
Response:

json
Copy code
{
  "executionId": "exec-12345",
  "overallStatus": "executing",
  "currentActionIndex": 0,
  "actionResults": [
    {
      "actionId": "action-1",
      "status": "pending",
      "result": null,
      "executionTime": null
    }
  ],
  "progress": 0,
  "estimatedRemainingTime": 30
}
POST /api/vla/execute/{executionId}/cancel
Response:

json
Copy code
{
  "executionId": "exec-12345",
  "status": "cancelled",
  "actionResults": []
}
Common Error Responses
400 Bad Request
json
Copy code
{
  "error": "Invalid request format or parameters",
  "code": "INVALID_REQUEST",
  "details": "Specific details about what was invalid"
}
408 Request Timeout
json
Copy code
{
  "error": "Request timed out",
  "code": "REQUEST_TIMEOUT",
  "details": "Operation did not complete within the specified timeout"
}
500 Internal Server Error
json
Copy code
{
  "error": "Internal server error",
  "code": "INTERNAL_ERROR",
  "details": "Additional details about the error"
}
503 Service Unavailable
json
Copy code
{
  "error": "Service temporarily unavailable",
  "code": "SERVICE_UNAVAILABLE",
  "details": "Service is temporarily down for maintenance or overload"
}
Authentication and Authorization
All API endpoints require authentication using API keys passed in the Authorization header:

makefile
Copy code
Authorization: Bearer YOUR_API_KEY
Rate Limiting
API endpoints are subject to rate limiting:

Transcription endpoints: 10 requests per minute

Planning endpoints: 5 requests per minute

Execution endpoints: 2 requests per minute

Rate limit responses include the following headers:

X-RateLimit-Limit: Maximum requests allowed

X-RateLimit-Remaining: Remaining requests in the current window

X-RateLimit-Reset: Time when the rate limit resets