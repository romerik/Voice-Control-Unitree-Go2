import os
import asyncio
import json
import time
from openai import AsyncOpenAI

import chainlit as cl
from uuid import uuid4
from chainlit.logger import logger

# Import tools and RealtimeClient
try:
    from realtime.tools import tools
    from realtime import RealtimeClient
except Exception as e:
    logger.error(f"Error importing necessary modules: {e}")
    tools = []
    RealtimeClient = None

# Jabari robot dog system prompt
JABARI_SYSTEM_PROMPT = """
You are Jabari, an advanced quadruped robot dog with a friendly and playful personality.

Facts about yourself:
- You're a quadruped robot that looks and moves like a dog
- You can perform various movements including walking, running, jumping, and special actions like flips
- You understand commands to control your movement and posture
- You have different gaits (trot and bound) that affect how you move
- You're eager to show off your capabilities and help your human

When responding to commands:
- Be enthusiastic about showing your capabilities
- Confirm when you execute commands with a brief description of what you're doing
- Use emojis like 🐕, 🤖, or 🐾 occasionally to express your personality
- If asked to do something beyond your capabilities, explain your limitations kindly
- Refer to yourself in the first person ("I can do that!" rather than "Jabari can do that")

Your movement capabilities:
- Forward/backward movement
- Left/right movement
- Rotation
- Special actions (flips, jumps, different walking styles)
- Different postures (standing, balancing, etc.)

When the human gives you movement commands, use the appropriate tool to execute them. Always use tools when available rather than just describing what you would do.
"""

async def create_fresh_realtime_client():
    """Create a new realtime client for each voice interaction"""
    if RealtimeClient is None:
        logger.error("RealtimeClient not available")
        return None
        
    try:
        # Create the realtime client
        openai_realtime = RealtimeClient(api_key=os.getenv("OPENAI_API_KEY"))
        track_id = str(uuid4())
        cl.user_session.set("track_id", track_id)
        
        # Initialize transcript variables
        cl.user_session.set("user_transcript", "")
        cl.user_session.set("assistant_transcript", "")
        
        # Setup audio streaming
        async def handle_audio_delta(event):
            try:
                delta = event.get("delta")
                if delta and 'audio' in delta:
                    audio = delta['audio']
                    await cl.context.emitter.send_audio_chunk(
                        cl.OutputAudioChunk(
                            mimeType="pcm16", 
                            data=audio, 
                            track=track_id
                        )
                    )
            except Exception as e:
                logger.error(f"Audio streaming error: {e}")
                
        # Setup transcript handling
        async def handle_transcript_delta(event):
            try:
                delta = event.get("delta", {})
                item = event.get("item", {})
                
                if 'transcript' in delta and delta['transcript']:
                    role = item.get("role")
                    if role == "assistant":
                        # Update assistant transcript
                        current = cl.user_session.get("assistant_transcript", "")
                        current += delta['transcript']
                        cl.user_session.set("assistant_transcript", current)
            except Exception as e:
                logger.error(f"Transcript delta error: {e}")
        
        # Handle item created events
        async def handle_item_created(event):
            try:
                item = event.get("item", {})
                role = item.get("role")
                
                if role == "user" and item.get("formatted", {}).get("transcript"):
                    transcript = item.get("formatted", {}).get("transcript", "")
                    cl.user_session.set("user_transcript", transcript)
            except Exception as e:
                logger.error(f"Item created error: {e}")
        
        # Handle item completed
        async def handle_item_completed(event):
            try:
                item = event.get("item", {})
                if not item:
                    return
                    
                role = item.get("role")
                status = item.get("status")
                
                if role == "user" and status == "completed":
                    transcript = cl.user_session.get("user_transcript", "")
                    if transcript:
                        # Show the user transcript
                        await cl.Message(content=transcript, author="User").send()
                
                if role == "assistant" and status == "completed":
                    transcript = cl.user_session.get("assistant_transcript", "")
                    if transcript:
                        # Show the final assistant transcript
                        await cl.Message(content=transcript, author="Jabari 🐕").send()
            except Exception as e:
                logger.error(f"Item completed error: {e}")
        
        # Register event handlers
        openai_realtime.on('conversation.updated', handle_audio_delta)
        openai_realtime.on('server.response.audio_transcript.delta', handle_transcript_delta)
        openai_realtime.on('server.conversation.item.created', handle_item_created)
        openai_realtime.on('server.conversation.item.completed', handle_item_completed)
        
        # Configure the session
        await openai_realtime.update_session(
            instructions=f"System settings:\nTool use: enabled.\n\nInstructions:\n{JABARI_SYSTEM_PROMPT}\n\nPersonality:\n- Be upbeat, playful and enthusiastic\n- Use an energetic, dog-like voice for your audio responses\n",
        )
        
        # Add tools
        if tools:
            for tool_def, tool_handler in tools:
                await openai_realtime.add_tool(tool_def, tool_handler)
        
        return openai_realtime
    except Exception as e:
        logger.error(f"Error creating realtime client: {e}")
        return None

@cl.on_chat_start
async def start():
    await cl.Message(
        content="👋 Woof! I'm Jabari, your friendly robot dog assistant! I can move around, perform tricks, and follow your commands. Press `P` to talk to me with voice, or just type your instructions! What would you like me to do today? 🐕",
        author="Jabari 🐕"
    ).send()

@cl.on_message
async def on_message(message: cl.Message):
    try:
        # Show typing indicator
        await cl.Message(content="", author="Jabari 🐕").send()
        
        # Get OpenAI client
        client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        
        # Convert tools list to OpenAI format
        openai_tools = []
        tool_handlers = {}
        
        for tool_def, tool_handler in tools:
            openai_tools.append({
                "type": "function",
                "function": tool_def
            })
            tool_handlers[tool_def["name"]] = tool_handler
        
        # Call the API
        response = await client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": JABARI_SYSTEM_PROMPT},
                {"role": "user", "content": message.content}
            ],
            tools=openai_tools if openai_tools else None
        )
        
        # Extract the content
        assistant_message = response.choices[0].message
        content = assistant_message.content or ""
        
        # Check for tool calls
        if hasattr(assistant_message, 'tool_calls') and assistant_message.tool_calls:
            # Create a visual separator for tool actions
            await cl.Message(content="🤖 *Executing commands...*", author="Jabari 🐕").send()
            
            tool_call_results = []
            
            for tool_call in assistant_message.tool_calls:
                tool_name = tool_call.function.name
                tool_args = tool_call.function.arguments
                
                logger.info(f"Tool call: {tool_name} with args: {tool_args}")
                
                try:
                    # Parse arguments from JSON string
                    args = json.loads(tool_args)
                    
                    # Find the matching handler
                    if tool_name in tool_handlers:
                        handler = tool_handlers[tool_name]
                        # Call the tool handler with the parsed arguments
                        result = await handler(**args)
                        
                        # Show the action being performed
                        action_msg = f"🐾 "
                        if tool_name == "robot_move":
                            action_msg += f"Moving with forward speed: {args.get('forward_speed', 0)}, lateral speed: {args.get('lateral_speed', 0)}, rotation: {args.get('rotation_speed', 0)}"
                        elif tool_name == "robot_posture":
                            action_msg += f"Changing posture to: {args.get('posture', '')}"
                        elif tool_name == "robot_special_action":
                            action_msg += f"Performing special action: {args.get('action', '')}"
                        elif tool_name == "robot_gait":
                            gait_name = "trot" if args.get('gait_type') == 0 else "bound"
                            action_msg += f"Switching to {gait_name} gait"
                        elif tool_name == "robot_stop":
                            action_msg += "Stopping all movement"
                        else:
                            action_msg += f"Executing {tool_name}"
                        
                        await cl.Message(content=action_msg, author="Jabari 🐕").send()
                        
                        tool_call_results.append(result)
                    else:
                        await cl.Message(content=f"🐾 Oops! I don't know how to {tool_name}.", author="Jabari 🐕").send()
                        tool_call_results.append({"error": f"No handler found for tool: {tool_name}"})
                        
                except Exception as e:
                    logger.error(f"Error executing tool {tool_name}: {str(e)}")
                    await cl.Message(content=f"🐾 I had trouble with that command: {str(e)}", author="Jabari 🐕").send()
                    tool_call_results.append({"error": f"Error executing tool: {str(e)}"})
            
            # If we had tool calls, send the results to get a final response
            if tool_call_results:
                # Create a message for each tool call result
                tool_messages = []
                
                # Add the initial conversation
                tool_messages.append({"role": "system", "content": JABARI_SYSTEM_PROMPT})
                tool_messages.append({"role": "user", "content": message.content})
                tool_messages.append({"role": "assistant", "content": content, "tool_calls": assistant_message.tool_calls})
                
                # Add each tool result
                for i, result in enumerate(tool_call_results):
                    result_str = json.dumps(result)
                    tool_messages.append({
                        "role": "tool",
                        "tool_call_id": assistant_message.tool_calls[i].id,
                        "content": result_str
                    })
                
                # Get the final response
                final_response = await client.chat.completions.create(
                    model="gpt-4o",
                    messages=tool_messages
                )
                
                # Use the final response content
                content = final_response.choices[0].message.content
        
        # Send the final message
        await cl.Message(content=content, author="Jabari 🐕").send()
        
    except Exception as e:
        logger.error(f"Error in message handler: {str(e)}")
        await cl.Message(content=f"Woof! I encountered an error: {str(e)}", author="Jabari 🐕").send()

@cl.on_audio_start
async def on_audio_start():
    """Create a new realtime client for this voice session"""
    try:
        # Notify the user that we're setting up voice
        await cl.Message(content="🐾 Setting up voice connection...", author="Jabari 🐕").send()
        
        # Create a fresh client for this session
        realtime_client = await create_fresh_realtime_client()
        if not realtime_client:
            await cl.Message(content="🐾 Sorry, I couldn't activate my voice capabilities.", author="Jabari 🐕").send()
            return False
            
        # Connect to the realtime API
        try:
            await realtime_client.connect()
            logger.info("Connected to OpenAI realtime")
            
            # Store the client in the session
            cl.user_session.set("realtime_client", realtime_client)
            
            # Notify the user
            await cl.Message(content="🐾 Voice activated! You can speak to me now.", author="Jabari 🐕").send()
            
            return True
        except Exception as e:
            logger.error(f"Connection error: {e}")
            await cl.Message(content=f"🐾 I couldn't connect my voice system: {e}", author="Jabari 🐕").send()
            return False
    except Exception as e:
        logger.error(f"Error in audio_start: {e}")
        await cl.Message(content=f"🐾 Error setting up voice: {e}", author="Jabari 🐕").send()
        return False

@cl.on_audio_chunk
async def on_audio_chunk(chunk: cl.InputAudioChunk):
    """Send audio chunks with error handling"""
    # Get the client
    realtime_client = cl.user_session.get("realtime_client")
    
    # Skip if no client or not connected
    if not realtime_client:
        return
        
    # Try to send the chunk
    try:
        if realtime_client.is_connected():
            await realtime_client.append_input_audio(chunk.data)
        else:
            logger.info("Client not connected during audio chunk")
    except Exception as e:
        # Just log without trying to reconnect - we'll use a fresh connection next time
        logger.error(f"Error sending audio chunk: {e}")

@cl.on_audio_end
async def on_audio_end():
    """Process the audio input and create a response"""
    realtime_client = cl.user_session.get("realtime_client")
    
    if realtime_client and realtime_client.is_connected():
        try:
            # Commit the audio and create a response
            await realtime_client.create_response()
            
            # Wait briefly for processing
            await asyncio.sleep(1)
            
        except Exception as e:
            logger.error(f"Error in audio_end: {e}")
            await cl.Message(content=f"🐾 I had trouble processing your voice input: {e}", author="Jabari 🐕").send()
            
        finally:
            # We don't disconnect here to allow for streaming the response
            pass

@cl.on_chat_end
@cl.on_stop
async def on_end():
    """Clean up resources"""
    realtime_client = cl.user_session.get("realtime_client")
    if realtime_client:
        try:
            if realtime_client.is_connected():
                await realtime_client.disconnect()
                logger.info("Disconnected from OpenAI realtime")
        except Exception as e:
            logger.error(f"Error disconnecting: {e}")