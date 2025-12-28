import React, { useState } from 'react';
import { authAPI, setAuthToken } from '../../utils/auth';
import '../../css/auth.css';

const SignupForm = ({ onSignupSuccess }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    firstName: '',
    lastName: '',
    experienceLevel: 'beginner',
    softwareSkills: {
      programming_languages: [],
      frameworks: [],
      tools: []
    },
    hardwareAccess: {
      development_kits: [],
      robotics_platforms: [],
      simulation_environments: []
    },
    learningPreferences: {
      content_types: [],
      learning_pace: 'moderate',
      preferred_topics: []
    }
  });

  const [errors, setErrors] = useState({});
  const [loading, setLoading] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSkillChange = (category, index, field, value) => {
    setFormData(prev => ({
      ...prev,
      softwareSkills: {
        ...prev.softwareSkills,
        [category]: prev.softwareSkills[category].map((skill, i) =>
          i === index ? { ...skill, [field]: value } : skill
        )
      }
    }));
  };

  const addSkill = (category) => {
    setFormData(prev => ({
      ...prev,
      softwareSkills: {
        ...prev.softwareSkills,
        [category]: [...prev.softwareSkills[category], { name: '', proficiency: 1 }]
      }
    }));
  };

  const removeSkill = (category, index) => {
    setFormData(prev => ({
      ...prev,
      softwareSkills: {
        ...prev.softwareSkills,
        [category]: prev.softwareSkills[category].filter((_, i) => i !== index)
      }
    }));
  };

  const handleHardwareChange = (category, value) => {
    setFormData(prev => ({
      ...prev,
      hardwareAccess: {
        ...prev.hardwareAccess,
        [category]: value.split(',').map(item => item.trim()).filter(Boolean)
      }
    }));
  };

  const validateForm = () => {
    const newErrors = {};

    if (!formData.email) newErrors.email = 'Email is required';
    if (!formData.password) newErrors.password = 'Password is required';
    if (formData.password.length < 8) newErrors.password = 'Password must be at least 8 characters';
    if (formData.password !== formData.confirmPassword) newErrors.confirmPassword = 'Passwords do not match';
    if (!formData.firstName) newErrors.firstName = 'First name is required';
    if (!formData.lastName) newErrors.lastName = 'Last name is required';

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!validateForm()) return;

    setLoading(true);

    try {
      const response = await authAPI.register({
        email: formData.email,
        password: formData.password,
        first_name: formData.firstName,
        last_name: formData.lastName,
        experience_level: formData.experienceLevel,
        software_skills: formData.softwareSkills,
        hardware_access: formData.hardwareAccess,
        learning_preferences: formData.learningPreferences
      });

      setAuthToken(response.data.access_token);
      onSignupSuccess && onSignupSuccess();
    } catch (error) {
      console.error('Registration error:', error);
      setErrors({ general: 'Registration failed. Please try again.' });
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2>Create Your Account</h2>
      {errors.general && <div className="error-message">{errors.general}</div>}

      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            className={errors.email ? 'error' : ''}
          />
          {errors.email && <div className="error-text">{errors.email}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            className={errors.password ? 'error' : ''}
          />
          {errors.password && <div className="error-text">{errors.password}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="confirmPassword">Confirm Password</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            className={errors.confirmPassword ? 'error' : ''}
          />
          {errors.confirmPassword && <div className="error-text">{errors.confirmPassword}</div>}
        </div>

        <div className="form-row">
          <div className="form-group">
            <label htmlFor="firstName">First Name</label>
            <input
              type="text"
              id="firstName"
              name="firstName"
              value={formData.firstName}
              onChange={handleChange}
              className={errors.firstName ? 'error' : ''}
            />
            {errors.firstName && <div className="error-text">{errors.firstName}</div>}
          </div>

          <div className="form-group">
            <label htmlFor="lastName">Last Name</label>
            <input
              type="text"
              id="lastName"
              name="lastName"
              value={formData.lastName}
              onChange={handleChange}
              className={errors.lastName ? 'error' : ''}
            />
            {errors.lastName && <div className="error-text">{errors.lastName}</div>}
          </div>
        </div>

        <div className="form-group">
          <label htmlFor="experienceLevel">Experience Level</label>
          <select
            id="experienceLevel"
            name="experienceLevel"
            value={formData.experienceLevel}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div className="form-section">
          <h3>Software Skills</h3>

          <div className="skills-category">
            <h4>Programming Languages</h4>
            {formData.softwareSkills.programming_languages.map((skill, index) => (
              <div key={index} className="skill-item">
                <input
                  type="text"
                  placeholder="Language name"
                  value={skill.name}
                  onChange={(e) => handleSkillChange('programming_languages', index, 'name', e.target.value)}
                />
                <select
                  value={skill.proficiency}
                  onChange={(e) => handleSkillChange('programming_languages', index, 'proficiency', parseInt(e.target.value))}
                >
                  {[1, 2, 3, 4, 5].map(level => (
                    <option key={level} value={level}>{level}</option>
                  ))}
                </select>
                <button type="button" onClick={() => removeSkill('programming_languages', index)}>Remove</button>
              </div>
            ))}
            <button type="button" onClick={() => addSkill('programming_languages')}>Add Language</button>
          </div>

          <div className="skills-category">
            <h4>Frameworks</h4>
            {formData.softwareSkills.frameworks.map((skill, index) => (
              <div key={index} className="skill-item">
                <input
                  type="text"
                  placeholder="Framework name"
                  value={skill.name}
                  onChange={(e) => handleSkillChange('frameworks', index, 'name', e.target.value)}
                />
                <select
                  value={skill.proficiency}
                  onChange={(e) => handleSkillChange('frameworks', index, 'proficiency', parseInt(e.target.value))}
                >
                  {[1, 2, 3, 4, 5].map(level => (
                    <option key={level} value={level}>{level}</option>
                  ))}
                </select>
                <button type="button" onClick={() => removeSkill('frameworks', index)}>Remove</button>
              </div>
            ))}
            <button type="button" onClick={() => addSkill('frameworks')}>Add Framework</button>
          </div>

          <div className="skills-category">
            <h4>Tools</h4>
            {formData.softwareSkills.tools.map((skill, index) => (
              <div key={index} className="skill-item">
                <input
                  type="text"
                  placeholder="Tool name"
                  value={skill.name}
                  onChange={(e) => handleSkillChange('tools', index, 'name', e.target.value)}
                />
                <select
                  value={skill.proficiency}
                  onChange={(e) => handleSkillChange('tools', index, 'proficiency', parseInt(e.target.value))}
                >
                  {[1, 2, 3, 4, 5].map(level => (
                    <option key={level} value={level}>{level}</option>
                  ))}
                </select>
                <button type="button" onClick={() => removeSkill('tools', index)}>Remove</button>
              </div>
            ))}
            <button type="button" onClick={() => addSkill('tools')}>Add Tool</button>
          </div>
        </div>

        <div className="form-section">
          <h3>Hardware Access</h3>

          <div className="form-group">
            <label>Development Kits (comma separated)</label>
            <input
              type="text"
              placeholder="e.g., Raspberry Pi, Arduino, Jetson Nano"
              value={formData.hardwareAccess.development_kits.join(', ')}
              onChange={(e) => handleHardwareChange('development_kits', e.target.value)}
            />
          </div>

          <div className="form-group">
            <label>Robotics Platforms (comma separated)</label>
            <input
              type="text"
              placeholder="e.g., ROS 2, NVIDIA Isaac, ROS 1"
              value={formData.hardwareAccess.robotics_platforms.join(', ')}
              onChange={(e) => handleHardwareChange('robotics_platforms', e.target.value)}
            />
          </div>

          <div className="form-group">
            <label>Simulation Environments (comma separated)</label>
            <input
              type="text"
              placeholder="e.g., Gazebo, Unity, Webots"
              value={formData.hardwareAccess.simulation_environments.join(', ')}
              onChange={(e) => handleHardwareChange('simulation_environments', e.target.value)}
            />
          </div>
        </div>

        <div className="form-group">
          <label htmlFor="learningPace">Learning Pace</label>
          <select
            id="learningPace"
            name="learningPace"
            value={formData.learningPreferences.learning_pace}
            onChange={(e) => setFormData(prev => ({
              ...prev,
              learningPreferences: {
                ...prev.learningPreferences,
                learning_pace: e.target.value
              }
            }))}
          >
            <option value="slow">Slow</option>
            <option value="moderate">Moderate</option>
            <option value="fast">Fast</option>
          </select>
        </div>

        <button type="submit" disabled={loading} className="submit-button">
          {loading ? 'Creating Account...' : 'Create Account'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;