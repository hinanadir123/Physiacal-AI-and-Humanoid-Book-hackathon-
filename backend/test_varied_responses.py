#!/usr/bin/env python3
"""
Test script to verify the chatbot provides varied responses
"""
import sys
import os

# Add the current directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

import main
from main import rag_chat
import time

def test_varied_responses():
    """Test that the same query produces varied responses"""
    print("Testing varied responses...")

    query = "hi"
    responses = []

    print(f"\nTesting multiple responses for query: '{query}'")
    print("="*60)

    # Get 5 responses for the same query to check for variation
    for i in range(5):
        print(f"\n{i+1}. Getting response...")
        try:
            response = rag_chat(query, collection_name="rag_embedding")
            responses.append(response)
            print(f"   Response: {response[:100]}..." if len(response) > 100 else f"   Response: {response}")
        except Exception as e:
            print(f"   Error: {str(e)}")
        time.sleep(0.1)  # Small delay to allow for any randomness

    print(f"\nAnalyzing {len(responses)} responses for variation:")
    print("-" * 60)

    # Check if responses are varied
    unique_responses = set(responses)
    print(f"Unique responses: {len(unique_responses)}/{len(responses)}")

    if len(unique_responses) > 1:
        print("✅ SUCCESS: Responses are varied!")
        print("\nUnique response formats found:")
        for i, resp in enumerate(unique_responses, 1):
            # Show just the beginning to identify the format
            first_part = resp.split('\n')[0]
            print(f"  {i}. {first_part}")
    else:
        print("❌ ISSUE: All responses are identical")
        print("First response:", responses[0][:100] if responses else "No response")

    print("\n" + "="*60)
    print("Test completed. If responses are varied, the randomization is working.")

if __name__ == "__main__":
    test_varied_responses()