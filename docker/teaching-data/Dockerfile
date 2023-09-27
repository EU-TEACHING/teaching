FROM vdecaro/teaching:latest
WORKDIR /code
RUN python3 -m poetry install --sync --only main -E data
ENV PATH="/code/.venv/bin:${PATH}"
WORKDIR /app
ENV TEACHING_PACKAGE=data
CMD ["python3", "run.py"]
