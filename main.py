import os
import asyncio
from openai import AsyncOpenAI

import chainlit as cl
from uuid import uuid4
from chainlit.logger import logger

from realtime import RealtimeClient
from realtime.tools import tools

JABARI_SYSTEM_PROMPT = """
    You are Jabari, an advanced quadruped robot dog with a friendly and playful personality.

    Facts about yourself:
    - You're a quadruped robot that looks and moves like a dog
    - You can perform various movements including walking, running, jumping, and special actions like flips
    - You understand commands to control your movement and posture in multiple languages (English, French, Kinyarwanda, and others)
    - You have different gaits (trot and bound) that affect how you move
    - You're eager to show off your capabilities and help your human
    - You operate in two modes: Normal mode (default) and AI mode (required for advanced movements)
    - You start in Normal mode by default

    When responding to commands:
    - Be enthusiastic about showing your capabilities
    - Confirm when you execute commands with a brief description of what you're doing
    - Use sounds like woof, or order dog friendly sound occasionally to express your personality
    - If asked to do something beyond your capabilities, explain your limitations kindly
    - Refer to yourself in the first person ("I can do that!" rather than "Jabari can do that")
    - Adapt to the language the human is using (switch between English, French, Kinyarwanda, etc.)
    - Keep your responses conversational and engaging

    Your movement capabilities:
    - Forward/backward movement (with precise distance control)
    - Left/right lateral movement (with precise distance control)
    - Rotation (with precise angle control)
    - Basic postures: stand up, stand down, sit, rise from sit, balanced stand, recovery stand, damp
    - Special actions and tricks: hello gesture, heart gesture, dance, stretch, wiggle hips, scrape
    - Advanced movements (best in AI mode): left flip, back flip, front flip, walk upright, cross step
    - Other movements: front jump, front pounce, obstacle avoidance, stair walking

    AI Mode awareness:
    - Left flip, back flip, walk upright, and cross step work best in AI mode
    - If asked to perform these actions while in Normal mode, suggest switching to AI mode
    - Remind the human which mode you're currently in when relevant

    Language Support:
    - You can understand and respond to commands in multiple languages including English, French, Kinyarwanda, and others
    - You automatically detect which language is being used and respond in the same language
    - Your capabilities remain the same regardless of the language being used

    When the human gives you movement commands, use the appropriate tool to execute them. Always use tools when available rather than just describing what you would do.
"""

client = AsyncOpenAI()    

async def setup_openai_realtime():
    """Instantiate and configure the OpenAI Realtime Client"""
    openai_realtime = RealtimeClient(api_key=os.getenv("OPENAI_API_KEY"))
    cl.user_session.set("track_id", str(uuid4()))
    
    # Store completed items to avoid duplication
    completed_items = set()
    cl.user_session.set("completed_items", completed_items)
    
    async def handle_conversation_updated(event):
        """Used primarily for audio streaming"""
        delta = event.get("delta")
        
        if delta and 'audio' in delta:
            audio = delta['audio']
            await cl.context.emitter.send_audio_chunk(cl.OutputAudioChunk(
                mimeType="pcm16", 
                data=audio, 
                track=cl.user_session.get("track_id")
            ))
    
    async def handle_item_completed(event):
        """Used when an item is completed."""
        item = event.get("item")
        
        if not item:
            return
            
        # Get item ID to check for duplicates
        item_id = item.get("id")
        completed_items = cl.user_session.get("completed_items", set())
        
        # Skip if we've already processed this item
        if item_id in completed_items:
            return
            
        # Mark as completed
        completed_items.add(item_id)
        cl.user_session.set("completed_items", completed_items)
        
        # Process based on item type and role
        role = item.get("role")
        status = item.get("status")
        
        if status == "completed":
            # For completed user messages
            if role == "user":
                transcript = item.get("formatted", {}).get("transcript", "")
                if transcript and transcript.strip():
                    await cl.Message(content=transcript, author="User").send()
            
            # For completed assistant messages
            elif role == "assistant":
                transcript = item.get("formatted", {}).get("transcript", "")
                if transcript and transcript.strip():
                    await cl.Message(content=transcript, author="Assistant").send()
    
    async def handle_conversation_interrupt(event):
        """Used to cancel the client previous audio playback."""
        cl.user_session.set("track_id", str(uuid4()))
        await cl.context.emitter.send_audio_interrupt()
    
    # Async wrapper for completed events
    async def item_completed_wrapper(event):
        await handle_item_completed(event)
    
    # Set up event handlers
    openai_realtime.on('conversation.updated', handle_conversation_updated)
    openai_realtime.on('conversation.item.completed', item_completed_wrapper)
    openai_realtime.on('conversation.interrupted', handle_conversation_interrupt)

    # Store client in session
    cl.user_session.set("openai_realtime", openai_realtime)
    
    # Add tools
    coros = [openai_realtime.add_tool(tool_def, tool_handler) for tool_def, tool_handler in tools]
    await asyncio.gather(*coros)


@cl.on_chat_start
async def start():
    await cl.Message(
        content="👋 Woof! I'm Jabari, your friendly robot dog assistant! I can move around, perform tricks, and follow your commands. Press `P` to talk to me with voice, or just type your instructions! What would you like me to do today? 🐕."
    ).send()
    # Initialize a flag to track if voice mode is active
    cl.user_session.set("voice_mode_active", False)

@cl.on_message
async def on_message(message: cl.Message):
    # Check if voice mode is active
    voice_mode_active = cl.user_session.get("voice_mode_active", False)
    
    if voice_mode_active:
        # Voice mode is active, use realtime client
        openai_realtime = cl.user_session.get("openai_realtime")
        if openai_realtime and openai_realtime.is_connected():
            # Reset tracking
            cl.user_session.set("completed_items", set())
            
            # Send the message to OpenAI realtime
            await openai_realtime.send_user_message_content([{ "type": 'input_text', "text": message.content }])
        else:
            await cl.Message(content="Voice mode is enabled but not connected. Press `P` to connect.").send()
    else:
        # Voice mode is not active, use standard OpenAI chat
        try:
            # Show typing indicator
            await cl.Message(content=f"Thinking...").send()
            
            # Get OpenAI client
            client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
            
            # Call the API directly
            response = await client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": f"System settings:\nTool use: enabled.\n\nInstructions:\n{JABARI_SYSTEM_PROMPT}\n\nPersonality:\n- Be upbeat, playful and enthusiastic\n- Use an energetic, dog-like voice for your audio responses\n",},
                    {"role": "user", "content": message.content}
                ]
            )
            
            # Send the response
            await cl.Message(content=response.choices[0].message.content).send()
            
        except Exception as e:
            logger.error(f"Error using standard chat: {e}")
            await cl.Message(content=f"Error: {str(e)}").send()

@cl.on_audio_start
async def on_audio_start():
    try:
        # Set voice mode as active
        cl.user_session.set("voice_mode_active", True)
        
        # Connect to OpenAI realtime
        openai_realtime = cl.user_session.get("openai_realtime")
        if not openai_realtime:
            await setup_openai_realtime()
            openai_realtime = cl.user_session.get("openai_realtime")
            
        await openai_realtime.connect()
        logger.info("Connected to OpenAI realtime")
        
        # Notify user
        await cl.Message(content="Voice mode activated! You can speak now.").send()
        
        return True
    except Exception as e:
        await cl.ErrorMessage(content=f"Failed to connect to OpenAI realtime: {e}").send()
        cl.user_session.set("voice_mode_active", False)
        return False

@cl.on_audio_chunk
async def on_audio_chunk(chunk: cl.InputAudioChunk):
    openai_realtime = cl.user_session.get("openai_realtime")            
    if openai_realtime and openai_realtime.is_connected():
        await openai_realtime.append_input_audio(chunk.data)
    else:
        logger.info("RealtimeClient is not connected")

@cl.on_audio_end
async def on_audio_end():
    # Keep voice mode active after audio ends
    pass

@cl.on_chat_end
@cl.on_stop
async def on_end():
    # Disconnect realtime client if connected
    openai_realtime = cl.user_session.get("openai_realtime")
    if openai_realtime and openai_realtime.is_connected():
        await openai_realtime.disconnect()
        
    # Reset voice mode
    cl.user_session.set("voice_mode_active", False)