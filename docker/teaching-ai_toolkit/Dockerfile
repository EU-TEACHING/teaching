FROM vdecaro/teaching:latest
WORKDIR /code
RUN python3 -m poetry install --sync --only main -E ai_toolkit
ENV PATH="/code/.venv/bin:${PATH}"
WORKDIR /app
ENV TEACHING_PACKAGE=ai_toolkit
CMD ["python3", "run.py"]