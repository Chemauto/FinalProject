#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM Core Module

Provides vision-language model capabilities:
- Image understanding
- Scene analysis
- Object detection
- Environment perception
"""

import yaml
import base64
from pathlib import Path
from typing import Dict, Any, Optional, List, Union
import os


class VLMCore:
    """Core VLM interface for vision tasks"""

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize VLM Core

        Args:
            config_path: Path to configuration file
        """
        if config_path is None:
            config_path = Path(__file__).parent.parent / "config" / "vlm_config.yaml"

        self.config = self._load_config(config_path)
        self.prompt_templates = self._load_prompts()

    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load VLM configuration"""
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        return {
            'provider': 'openai',  # openai, anthropic, or local
            'model': 'gpt-4-vision-preview',
            'api_key': os.getenv('OPENAI_API_KEY'),
            'max_tokens': 1000
        }

    def _load_prompts(self) -> Dict[str, str]:
        """Load prompt templates from prompts/ directory"""
        prompts = {}
        prompts_dir = Path(__file__).parent / "prompts"

        if prompts_dir.exists():
            for prompt_file in prompts_dir.glob("*.yaml"):
                with open(prompt_file, 'r') as f:
                    prompts[prompt_file.stem] = yaml.safe_load(f)

        return prompts

    def get_prompt(self, prompt_name: str, **kwargs) -> str:
        """
        Get a prompt template and fill in variables

        Args:
            prompt_name: Name of the prompt template
            **kwargs: Variables to fill in the template

        Returns:
            Filled prompt string
        """
        template = self.prompt_templates.get(prompt_name, {})

        if isinstance(template, dict):
            prompt_text = template.get('prompt', '')
        else:
            prompt_text = str(template)

        return prompt_text.format(**kwargs)

    def analyze_image(self, image_path: str, task: str = "describe") -> Dict[str, Any]:
        """
        Analyze an image with VLM

        Args:
            image_path: Path to image file
            task: Type of analysis (describe, detect_objects, navigate, etc.)

        Returns:
            Analysis result as dictionary
        """
        prompt = self.get_prompt(task, image_path=image_path)

        provider = self.config.get('provider', 'openai')

        if provider == 'openai':
            return self._analyze_with_openai(image_path, prompt)
        elif provider == 'anthropic':
            return self._analyze_with_anthropic(image_path, prompt)
        else:
            raise ValueError(f"Unknown provider: {provider}")

    def _encode_image(self, image_path: str) -> str:
        """Encode image to base64"""
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    def _analyze_with_openai(self, image_path: str, prompt: str) -> Dict[str, Any]:
        """Analyze image using OpenAI GPT-4 Vision"""
        try:
            from openai import OpenAI
            client = OpenAI(api_key=self.config.get('api_key'))

            base64_image = self._encode_image(image_path)

            response = client.chat.completions.create(
                model=self.config.get('model', 'gpt-4-vision-preview'),
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{base64_image}"
                                }
                            }
                        ]
                    }
                ],
                max_tokens=self.config.get('max_tokens', 1000)
            )

            result = response.choices[0].message.content

            return {
                'success': True,
                'result': result,
                'provider': 'openai'
            }
        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'provider': 'openai'
            }

    def _analyze_with_anthropic(self, image_path: str, prompt: str) -> Dict[str, Any]:
        """Analyze image using Anthropic Claude"""
        try:
            from anthropic import Anthropic
            client = Anthropic(api_key=self.config.get('api_key'))

            base64_image = self._encode_image(image_path)

            response = client.messages.create(
                model=self.config.get('model', 'claude-3-opus-20240229'),
                max_tokens=self.config.get('max_tokens', 1000),
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image",
                                "source": {
                                    "type": "base64",
                                    "media_type": "image/jpeg",
                                    "data": base64_image
                                }
                            }
                        ]
                    }
                ]
            )

            result = response.content[0].text

            return {
                'success': True,
                'result': result,
                'provider': 'anthropic'
            }
        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'provider': 'anthropic'
            }

    def perceive_environment(self, image_path: str) -> Dict[str, Any]:
        """
        Perceive robot environment from camera image

        Args:
            image_path: Path to camera image

        Returns:
            Environment perception results
        """
        prompt = self.get_prompt('perceive_environment')

        return self.analyze_image(image_path, task='perceive_environment')

    def detect_obstacles(self, image_path: str) -> List[Dict[str, Any]]:
        """
        Detect obstacles in the environment

        Args:
            image_path: Path to camera image

        Returns:
            List of detected obstacles with positions
        """
        result = self.analyze_image(image_path, task='detect_obstacles')

        # Parse result to extract obstacle information
        # This would need proper parsing logic based on the VLM response format

        return result.get('obstacles', [])

    def suggest_navigation(self, image_path: str, goal: str) -> Dict[str, Any]:
        """
        Suggest navigation action based on visual input

        Args:
            image_path: Path to camera image
            goal: Navigation goal description

        Returns:
            Navigation suggestion
        """
        prompt = self.get_prompt('suggest_navigation', goal=goal)

        return self.analyze_image(image_path, task='suggest_navigation')
